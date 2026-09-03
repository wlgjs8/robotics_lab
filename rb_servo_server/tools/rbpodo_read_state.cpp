// READ-ONLY rbpodo controller state reader.
//
// WHY THIS EXISTS, AND WHY IT LINKS ONLY rbpodo
// =============================================
// Bring-up questions like "is the stand model right?" are answered by parking the
// arms by hand against a known surface and reading the joints back. The reading
// must be provably incapable of commanding motion, because it is run while a person
// is standing inside the workspace holding the arm.
//
// The rbpodo SDK splits this cleanly: rb::podo::Cobot owns every command entry point
// (move_j / move_l / servo_j / set_operation_mode / reset ...), while
// rb::podo::CobotData owns the read-only data port (default 5001) and exposes exactly
// one call, request_data(). This translation unit constructs ONLY CobotData and never
// names Cobot, and its CMake target links ONLY rbpodo::rbpodo -- not rb_servo_core --
// so the servo loop, the dispatcher and the backend are not in the binary at all.
// That is checkable after the fact:
//
//     nm -C rb_servo_server/build/rbpodo_read_state | grep -E 'move_j|servo_j|Cobot<'
//
// must print nothing. Keep it that way: do not link rb_servo_core into this target and
// do not add a "just one" motion helper here. Use scripts/send_*.py for anything that
// moves, so the two categories never share a binary.
//
// The Python sibling (scripts/rbpodo_state_dump.py) needs the rbpodo Python module,
// which is not installed in this workspace's venv (2026-09-02); this C++ tool uses the
// same SDK the server already links, so it works wherever the server builds.
//
// Usage:
//   rbpodo_read_state 172.28.60.200 172.28.60.201
//   rbpodo_read_state --json --samples 5 172.28.60.200
//
// Fields (raw controller units -- degrees, and millimetres for tcp_pos):
//   jnt_ang   measured joint angles
//   jnt_ref   the controller's own reference readback
//   tcp_pos   the controller's TCP pose in the ARM BASE frame (x,y,z mm, rx,ry,rz deg,
//             ZYX euler). This carries whatever tool offset is configured in the control
//             box, which is NOT necessarily the gripper tip -- on 2026-09-02 it measured
//             140.28 mm from link6 along the tool axis while the URDF attachment_site is
//             at 96.7 mm.
#include <rbpodo/rbpodo.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int kJoints = 6;
constexpr int kDataPort = 5001;

// rb::podo::CobotData's constructor connects, and that connect is NOT bounded by the
// request_data timeout: a wrong IP or a powered-down box hangs the tool forever (seen
// 2026-09-02 on a typo'd address). Probe reachability first with a bounded non-blocking
// connect. This only opens and closes a TCP connection -- it writes no bytes, so the
// read-only property is unchanged.
bool dataPortReachable(const std::string& ip, double timeout_sec, std::string* why) {
    ::sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(kDataPort);
    if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        *why = "not a valid IPv4 address";
        return false;
    }
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        *why = "socket() failed";
        return false;
    }
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    bool ok = false;
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
        ok = true;
    } else if (errno == EINPROGRESS) {
        ::fd_set wset;
        FD_ZERO(&wset);
        FD_SET(fd, &wset);
        ::timeval tv{};
        tv.tv_sec = static_cast<time_t>(timeout_sec);
        tv.tv_usec = static_cast<suseconds_t>((timeout_sec - tv.tv_sec) * 1e6);
        if (::select(fd + 1, nullptr, &wset, nullptr, &tv) > 0) {
            int err = 0;
            ::socklen_t len = sizeof(err);
            ok = ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) == 0 && err == 0;
            if (!ok) *why = std::string("connect failed: ") + std::strerror(err);
        } else {
            *why = "connect timed out (box powered down, wrong IP, or no route)";
        }
    } else {
        *why = std::string("connect failed: ") + std::strerror(errno);
    }
    ::close(fd);
    return ok;
}

// Documented valid bits, mirroring kRbpodo*Mask in src/robot/rbpodo_backend.cpp.
// Stillness bound, chosen by what it costs where it is used. This tool exists to
// certify hand-parked poses for geometry work whose own precision is a few mm, and
// an ACTIVATED arm never reads perfectly constant -- it dithers as the servo holds
// it. 0.05 deg is about 0.9 mm at the TCP of a 1 m arm, an order below that
// precision, while still an order above the ~0.01 deg dither measured on both arms
// once activated (2026-09-02).
constexpr double kStillSpreadDeg = 0.05;

constexpr int kCollisionOccurMask = 0b11;
constexpr int kSelfCollisionMask = 0b11;
constexpr int kStatusCodeMask = 0b111111;

struct Sample {
    double jnt_ang[kJoints];
    double jnt_ref[kJoints];
    double tcp_pos[kJoints];
    double eft[kJoints];   // external F/T, IN THE SENSOR'S OWN AXES (see below)
    int init_state_info;   // 0..6; 6 = activation done
    int init_error;
    int robot_state;
    int sos;
    int ems;
    int soft_estop;
    int collision_occur;
    int self_collision;
};

// A hand-parked arm must read identically across samples. Report the spread so the
// operator can tell "parked" from "drifting/being held" before trusting the numbers.
struct Stability {
    double max_sample_spread_deg = 0.0;  // max over joints of (max-min) across samples
    double max_ref_error_deg = 0.0;      // max over joints/samples of |jnt_ang - jnt_ref|
};

Stability stabilityOf(const std::vector<Sample>& s) {
    Stability out;
    if (s.empty()) return out;
    for (int j = 0; j < kJoints; ++j) {
        double lo = s.front().jnt_ang[j];
        double hi = lo;
        for (const auto& x : s) {
            lo = std::min(lo, x.jnt_ang[j]);
            hi = std::max(hi, x.jnt_ang[j]);
            out.max_ref_error_deg =
                std::max(out.max_ref_error_deg, std::fabs(x.jnt_ang[j] - x.jnt_ref[j]));
        }
        out.max_sample_spread_deg = std::max(out.max_sample_spread_deg, hi - lo);
    }
    return out;
}

// A steady readback is NOT enough to trust the numbers. After the v8.9.1 firmware
// flash on 2026-09-02 the left box returned all-zero joints with tcp_pos at the
// fully-extended zero pose, perfectly stable across samples -- an UNACTIVATED
// controller reporting its default, which the old spread-only test called PARKED.
// Geometry work needs an activated, fault-free, actually-still arm, so check all
// three, and treat an exactly-zero joint vector as the default-state tell it is.
std::string verdict(const Sample& s, const Stability& st) {
    if (s.init_state_info != 6)
        return "NOT ACTIVATED (init_state " + std::to_string(s.init_state_info) +
               "/6) -- these are default values, not a pose";
    bool all_zero = true;
    for (int j = 0; j < kJoints; ++j)
        if (s.jnt_ang[j] != 0.0) { all_zero = false; break; }
    if (all_zero) return "ALL-ZERO joints -- almost certainly a default readback, not a pose";
    if (s.init_error) return "INIT ERROR " + std::to_string(s.init_error);
    if (s.sos || s.ems || s.soft_estop || s.collision_occur || s.self_collision)
        return "CONTROLLER FAULT LATCHED -- clear it before trusting these numbers";
    // STILLNESS is the sample spread, not the reference error. jnt_ang is the measured
    // encoder angle, which is what geometry consumes, and |jnt_ang - jnt_ref| is the
    // servo's static droop while it holds the arm up -- real, but it does not make the
    // measurement wrong. Gating on it was an artifact of first using this tool on arms
    // that were hand-posed with the brakes holding, where ref == ang exactly; once both
    // arms were activated on 2026-09-02 the right one held at 0.0117 deg and got called
    // NOT SETTLED while sitting still to 0.0004 deg. The loose bound below still catches
    // an arm that is actually being driven somewhere.
    if (st.max_sample_spread_deg >= kStillSpreadDeg)
        return "MOVING (" + std::to_string(st.max_sample_spread_deg) +
               " deg across samples) -- do not use these numbers for geometry";
    if (st.max_ref_error_deg >= 0.5)
        return "TRACKING A COMMAND (|jnt_ang-jnt_ref| " +
               std::to_string(st.max_ref_error_deg) + " deg) -- not a parked pose";
    return "PARKED (activated, fault-free, still: safe to use for geometry checks)";
}

void printJointArray(const char* label, const double* v, const char* unit) {
    std::printf("    %-9s", label);
    for (int j = 0; j < kJoints; ++j) std::printf("%12.4f", v[j]);
    std::printf("   [%s]\n", unit);
}

void printJson(const char* label, const double* v, bool last) {
    std::printf("      \"%s\": [", label);
    for (int j = 0; j < kJoints; ++j) std::printf("%s%.6f", j ? ", " : "", v[j]);
    std::printf("]%s\n", last ? "" : ",");
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> ips;
    int samples = 3;
    double interval_sec = 0.05;
    double timeout_sec = 1.0;
    bool json = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* what) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s requires a value\n", what);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--json") json = true;
        else if (a == "--samples") samples = std::atoi(next("--samples"));
        else if (a == "--interval-sec") interval_sec = std::atof(next("--interval-sec"));
        else if (a == "--timeout-sec") timeout_sec = std::atof(next("--timeout-sec"));
        else if (a == "-h" || a == "--help") {
            std::printf(
                "READ-ONLY rbpodo state reader (never commands motion).\n"
                "usage: %s [--json] [--samples N] [--interval-sec S] [--timeout-sec S] <ip> [ip ...]\n",
                argv[0]);
            return 0;
        } else if (!a.empty() && a[0] == '-') {
            std::fprintf(stderr, "unknown option: %s\n", a.c_str());
            return 2;
        } else {
            ips.push_back(a);
        }
    }
    if (ips.empty()) {
        std::fprintf(stderr,
                     "no controller IP given\n"
                     "usage: %s [--json] [--samples N] <ip> [ip ...]\n",
                     argv[0]);
        return 2;
    }
    if (samples < 1) {
        std::fprintf(stderr, "--samples must be >= 1\n");
        return 2;
    }

    if (json) std::printf("{\n  \"read_only\": true,\n  \"controllers\": [\n");
    int failures = 0;

    for (std::size_t n = 0; n < ips.size(); ++n) {
        const std::string& ip = ips[n];
        std::vector<Sample> got;
        std::string error;
        try {
            if (std::string why; !dataPortReachable(ip, timeout_sec, &why)) {
                throw std::runtime_error("data port " + std::to_string(kDataPort) +
                                         " unreachable: " + why);
            }
            // ONLY the data-port class. See the header comment.
            const rb::podo::CobotData data(ip);
            for (int s = 0; s < samples; ++s) {
                if (s) std::this_thread::sleep_for(
                    std::chrono::duration<double>(interval_sec));
                const auto st = data.request_data(timeout_sec);
                if (!st.has_value()) {
                    error = "timeout waiting for a state frame";
                    break;
                }
                Sample smp{};
                for (int j = 0; j < kJoints; ++j) {
                    smp.jnt_ang[j] = st->sdata.jnt_ang[j];
                    smp.jnt_ref[j] = st->sdata.jnt_ref[j];
                    smp.tcp_pos[j] = st->sdata.tcp_pos[j];
                }
                smp.eft[0] = st->sdata.eft_fx; smp.eft[1] = st->sdata.eft_fy;
                smp.eft[2] = st->sdata.eft_fz; smp.eft[3] = st->sdata.eft_mx;
                smp.eft[4] = st->sdata.eft_my; smp.eft[5] = st->sdata.eft_mz;
                smp.init_state_info = st->sdata.init_state_info;
                smp.init_error = st->sdata.init_error;
                smp.robot_state = st->sdata.robot_state;
                smp.sos = st->sdata.op_stat_sos_flag & kStatusCodeMask;
                smp.ems = st->sdata.op_stat_ems_flag & kStatusCodeMask;
                smp.soft_estop = st->sdata.op_stat_soft_estop_occur & kStatusCodeMask;
                smp.collision_occur = st->sdata.op_stat_collision_occur & kCollisionOccurMask;
                smp.self_collision = st->sdata.op_stat_self_collision & kSelfCollisionMask;
                got.push_back(smp);
            }
        } catch (const std::exception& e) {
            error = e.what();
        }

        const bool ok = error.empty() && !got.empty();
        if (!ok) ++failures;
        const Stability st = stabilityOf(got);

        if (json) {
            std::printf("    {\n      \"ip\": \"%s\",\n      \"ok\": %s,\n",
                        ip.c_str(), ok ? "true" : "false");
            if (!ok) {
                std::printf("      \"error\": \"%s\"\n", error.c_str());
            } else {
                printJson("jnt_ang_deg", got.back().jnt_ang, false);
                printJson("jnt_ref_deg", got.back().jnt_ref, false);
                printJson("tcp_pos", got.back().tcp_pos, false);
                printJson("eft_raw", got.back().eft, false);
                std::printf("      \"samples\": %d,\n", static_cast<int>(got.size()));
                std::printf("      \"max_sample_spread_deg\": %.6f,\n", st.max_sample_spread_deg);
                std::printf("      \"max_ref_error_deg\": %.6f\n", st.max_ref_error_deg);
            }
            std::printf("    }%s\n", n + 1 == ips.size() ? "" : ",");
        } else {
            std::printf("=== %s\n", ip.c_str());
            if (!ok) {
                std::printf("    ERROR: %s\n", error.empty() ? "no samples" : error.c_str());
            } else {
                printJointArray("jnt_ang", got.back().jnt_ang, "deg");
                printJointArray("jnt_ref", got.back().jnt_ref, "deg");
                printJointArray("tcp_pos", got.back().tcp_pos, "mm,deg (arm base frame)");
                // UNINTERPRETED, exactly as rbpodo_backend carries it: these are the
                // SENSOR's own axes, not flange- or tool-aligned. Turning them into a
                // wrench needs force_torque.<arm>.axes from the config, which is a
                // per-machine measurement. Printed raw so a direction check does not
                // depend on that mapping being right -- which is the thing being checked.
                printJointArray("eft(raw)", got.back().eft, "N,Nm  SENSOR AXES, biased");
                const Sample& last = got.back();
                std::printf("    init_state %d/6%s  robot_state %d  sos %d  ems %d  soft_estop %d"
                            "  collision %d  self_collision %d\n",
                            last.init_state_info,
                            last.init_state_info == 6 ? " (activation done)" : " (NOT ACTIVATED)",
                            last.robot_state, last.sos, last.ems, last.soft_estop,
                            last.collision_occur, last.self_collision);
                std::printf("    %d samples: joint spread %.4f deg, |jnt_ang-jnt_ref| %.4f deg\n",
                            static_cast<int>(got.size()), st.max_sample_spread_deg,
                            st.max_ref_error_deg);
                std::printf("    -> %s\n", verdict(last, st).c_str());
            }
        }
    }

    if (json) std::printf("  ]\n}\n");
    return failures ? 1 : 0;
}
