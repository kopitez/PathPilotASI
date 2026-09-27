#include <plugin.h>
#include <common.h>
#include <CPathFind.h>
#include <CPathNode.h>
#include <CCarAI.h>
#include <CVehicle.h>
#include <CTimer.h>
#include <eCarMission.h>
#include <windows.h>
#include <algorithm>
#include <cmath>

using namespace plugin;

namespace {

struct CheckpointInfo {
    CVector pos{};
    float radius{};
    bool valid{};
};

static unsigned GetSampVersion() {
    const auto samp = reinterpret_cast<uintptr_t>(GetModuleHandleA("samp.dll"));
    if (!samp) return 0;
    const auto sig = *reinterpret_cast<uint32_t*>(samp + 296);
    switch (sig) {
    case 1430451322u: return 1;
    case 1505954964u: return 2;
    case 1516908848u: return 3;
    case 1544241731u: return 4;
    case 1574307533u: return 5;
    case 1620356267u: return 6;
    case 1668465566u: return 7;
    default: return 0;
    }
}

static CheckpointInfo ReadCurrentCheckpoint() {
    CheckpointInfo out{};
    const auto samp = reinterpret_cast<uintptr_t>(GetModuleHandleA("samp.dll"));
    if (!samp) return out;
    const auto version = GetSampVersion();
    if (!version) return out;

    uintptr_t cp = 0;
    if (version == 1) {
        cp = *reinterpret_cast<uintptr_t*>(samp + 2203916);
        if (!cp) return out;
        const uint32_t flagA = *reinterpret_cast<uint32_t*>(cp + 68);
        const uint32_t flagB = *reinterpret_cast<uint32_t*>(cp + 73);
        if (flagB == 1) {
            out.pos.x = *reinterpret_cast<float*>(cp + 44);
            out.pos.y = *reinterpret_cast<float*>(cp + 48);
            out.pos.z = *reinterpret_cast<float*>(cp + 52);
            out.radius = *reinterpret_cast<float*>(cp + 68);
            out.valid = true;
        } else if (flagA == 1) {
            out.pos.x = *reinterpret_cast<float*>(cp + 12);
            out.pos.y = *reinterpret_cast<float*>(cp + 16);
            out.pos.z = *reinterpret_cast<float*>(cp + 20);
            out.radius = (*reinterpret_cast<float*>(cp + 24) + *reinterpret_cast<float*>(cp + 28)) * 0.5f;
            out.valid = true;
        }
        return out;
    }

    const uintptr_t offsets[] = { 2203924, 2804284, 2550004, 2550308, 2550308, 2550700 };
    const unsigned idx = version - 2;
    cp = *reinterpret_cast<uintptr_t*>(samp + offsets[idx]);
    if (!cp) return out;

    const uint32_t flagA = *reinterpret_cast<uint32_t*>(cp + 36);
    const uint32_t flagB = *reinterpret_cast<uint32_t*>(cp + 41);
    if (flagB == 1) {
        out.pos.x = *reinterpret_cast<float*>(cp + 12);
        out.pos.y = *reinterpret_cast<float*>(cp + 16);
        out.pos.z = *reinterpret_cast<float*>(cp + 20);
        out.radius = *reinterpret_cast<float*>(cp + 36);
        out.valid = true;
    } else if (flagA == 1) {
        out.pos.x = *reinterpret_cast<float*>(cp + 53);
        out.pos.y = *reinterpret_cast<float*>(cp + 57);
        out.pos.z = *reinterpret_cast<float*>(cp + 61);
        out.radius = (*reinterpret_cast<float*>(cp + 65) + *reinterpret_cast<float*>(cp + 69)) * 0.5f;
        out.valid = true;
    }
    return out;
}

static float Dist2D2(const CVector& a, const CVector& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return dx * dx + dy * dy;
}

class PathPilot {
public:
    PathPilot() {
        Events::gameProcessEvent += [this] { Process(); };
    }

private:
    static constexpr int MAX_ROUTE = 64;
    bool m_hasTarget = false;
    CVector m_finalTarget{};
    float m_finalRadius = 0.0f;
    CNodeAddress m_route[MAX_ROUTE]{};
    int m_routeCount = 0;
    int m_routeIndex = 0;
    CVehicle* m_lastVehicle = nullptr;
    CVector m_lastPosition{};
    unsigned m_lastMoveCheck = 0;
    unsigned m_stuckSince = 0;
    unsigned m_lastRouteBuild = 0;
    unsigned m_lastCommand = 0;

    static bool IsAutoDriveMission(CVehicle* vehicle) {
        const auto mission = vehicle->m_autoPilot.m_nCarMission;
        return mission == MISSION_GOTOCOORDINATES ||
               mission == MISSION_GOTOCOORDINATES_STRAIGHTLINE ||
               mission == MISSION_GOTOCOORDINATES_ACCURATE ||
               mission == MISSION_GOTOCOORDINATES_STRAIGHTLINE_ACCURATE;
    }

    void Reset() {
        m_hasTarget = false;
        m_routeCount = 0;
        m_routeIndex = 0;
        m_lastVehicle = nullptr;
        m_stuckSince = 0;
    }

    bool BuildRoute(CVehicle* vehicle, const CVector& target) {
        CNodeAddress invalid{};
        short count = 0;
        float distance = 0.0f;
        CNodeAddress result[MAX_ROUTE]{};
        ThePaths.DoPathSearch(
            PATH_TYPE_VEH,
            vehicle->GetPosition(),
            invalid,
            target,
            result,
            &count,
            MAX_ROUTE,
            &distance,
            100000.0f,
            &invalid,
            100000.0f,
            false,
            invalid,
            false,
            false
        );
        if (count <= 0) {
            m_routeCount = 0;
            m_routeIndex = 0;
            return false;
        }
        m_routeCount = std::min<int>(count, MAX_ROUTE);
        for (int i = 0; i < m_routeCount; ++i) m_route[i] = result[i];
        m_routeIndex = 0;
        m_lastRouteBuild = CTimer::m_snTimeInMilliseconds;
        return true;
    }

    CVector NodePosition(int index) const {
        if (index < 0 || index >= m_routeCount) return {};
        auto* node = ThePaths.GetPathNode(m_route[index]);
        return node ? node->GetNodeCoors() : CVector{};
    }

    void AdvanceRouteIndex(const CVector& vehiclePos) {
        while (m_routeIndex < m_routeCount) {
            if (Dist2D2(vehiclePos, NodePosition(m_routeIndex)) <= 14.0f * 14.0f) ++m_routeIndex;
            else break;
        }
    }

    void IssueRouteCommand(CVehicle* vehicle, const CVector& target) {
        CCarAI::GetCarToGoToCoors(vehicle, const_cast<CVector*>(&target), DRIVINGSTYLE_STOP_FOR_CARS, false);
        m_lastCommand = CTimer::m_snTimeInMilliseconds;
    }

    void Process() {
        auto* vehicle = FindPlayerVehicle(-1, false);
        if (!vehicle || !IsAutoDriveMission(vehicle)) { Reset(); return; }
        const auto cp = ReadCurrentCheckpoint();
        if (!cp.valid) { Reset(); return; }

        const auto now = CTimer::m_snTimeInMilliseconds;
        const auto vehiclePos = vehicle->GetPosition();
        const float checkpointDist2 = Dist2D2(vehiclePos, cp.pos);

        if (!m_hasTarget || Dist2D2(cp.pos, m_finalTarget) > 12.0f * 12.0f || vehicle != m_lastVehicle) {
            m_hasTarget = true;
            m_finalTarget = cp.pos;
            m_finalRadius = cp.radius;
            m_routeCount = 0;
            m_routeIndex = 0;
            m_stuckSince = 0;
            m_lastVehicle = vehicle;
        }

        const float finishRadius = std::max(8.0f, m_finalRadius + 3.0f);
        if (checkpointDist2 <= finishRadius * finishRadius) {
            IssueRouteCommand(vehicle, m_finalTarget);
            return;
        }

        if (now - m_lastMoveCheck >= 500) {
            const float moved2 = Dist2D2(vehiclePos, m_lastPosition);
            if (m_lastMoveCheck != 0 && moved2 < 0.35f * 0.35f) {
                if (!m_stuckSince) m_stuckSince = now;
            } else {
                m_stuckSince = 0;
            }
            m_lastPosition = vehiclePos;
            m_lastMoveCheck = now;
        }

        const bool stuck = m_stuckSince && now - m_stuckSince >= 1800;
        if (stuck || m_routeCount == 0 || m_routeIndex >= m_routeCount || now - m_lastRouteBuild >= 4000) {
            if (!BuildRoute(vehicle, m_finalTarget)) {
                if (now - m_lastCommand >= 750) IssueRouteCommand(vehicle, m_finalTarget);
                m_stuckSince = 0;
                return;
            }
            m_stuckSince = 0;
        }

        AdvanceRouteIndex(vehiclePos);
        if (m_routeIndex >= m_routeCount) { IssueRouteCommand(vehicle, m_finalTarget); return; }
        int lookAhead = std::min(m_routeIndex + 3, m_routeCount - 1);
        const auto nodeTarget = NodePosition(lookAhead);
        if (Dist2D2(vehiclePos, nodeTarget) < 20.0f * 20.0f) { AdvanceRouteIndex(vehiclePos); return; }
        if (now - m_lastCommand >= 600) IssueRouteCommand(vehicle, nodeTarget);
    }
};

PathPilot gPathPilot;

} // namespace
