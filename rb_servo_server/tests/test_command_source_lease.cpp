#include <cmath>
#include <iostream>
#include <string>

#include "rb_servo/core/clock.hpp"
#include "rb_servo/network/command_server.hpp"

#define RB_CHECK(expr) do { \
    if (!(expr)) { \
        std::cerr << "CHECK failed at " << __FILE__ << ":" << __LINE__ << ": " #expr "\n"; \
        return false; \
    } \
} while (0)

namespace {

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

bool testAcquireLeaseAndExpiration() {
    rb_servo::NetworkConfig network;
    network.command_source_enforce_lease = true;
    network.command_source_lease_timeout_sec = 1.0;
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    rb_servo::DualArmCommand out;
    const uint64_t now = rb_servo::nowSteadyNs();

    RB_CHECK(server.parseMessage(
        R"({"seq":1,"mode":"AcquireLease","source_id":"policy_runner","session_id":"policy-session"})",
        now,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(out.lease.active);
    RB_CHECK(out.lease.enforce_lease);
    RB_CHECK(out.lease.source_id == "policy_runner");
    RB_CHECK(out.lease.session_id == "policy-session");
    RB_CHECK(!out.lease.lease_token.empty());
    const std::string policy_token = out.lease.lease_token;

    RB_CHECK(!server.parseMessage(
        R"({"seq":1,"mode":"AcquireLease","source_id":"rb_gui","session_id":"gui-session"})",
        now + 1,
        &out
    ));
    RB_CHECK(contains(server.lastRejectReason(), "command_source_lease_conflict"));

    RB_CHECK(server.parseMessage(
        R"({"seq":2,"mode":"AcquireLease","source_id":"rb_gui","session_id":"gui-session"})",
        now + 1'100'000'000ULL,
        &out
    ));
    RB_CHECK(out.lease.active);
    RB_CHECK(out.lease.source_id == "rb_gui");
    RB_CHECK(out.lease.session_id == "gui-session");
    RB_CHECK(out.lease.lease_token != policy_token);
    return true;
}

bool testWrongTokenRejected() {
    rb_servo::NetworkConfig network;
    network.command_source_enforce_lease = true;
    network.command_source_lease_timeout_sec = 1.0;
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    rb_servo::DualArmCommand out;
    const uint64_t now = rb_servo::nowSteadyNs();

    RB_CHECK(server.parseMessage(
        R"({"seq":1,"mode":"AcquireLease","source_id":"policy_runner","session_id":"policy-session","lease_token":"good-token"})",
        now,
        &out
    ));
    RB_CHECK(out.lease.lease_token == "good-token");

    RB_CHECK(!server.parseMessage(
        R"({"seq":2,"mode":"JointTarget","source_id":"policy_runner","session_id":"policy-session","lease_token":"wrong-token","q_target_deg":[1,2,3,4,5,6]})",
        now + 1,
        &out
    ));
    RB_CHECK(contains(server.lastRejectReason(), "command_source_lease_token_mismatch"));

    RB_CHECK(!server.parseMessage(
        R"({"seq":2,"mode":"AcquireLease","source_id":"policy_runner","session_id":"policy-session","lease_token":"wrong-token"})",
        now + 2,
        &out
    ));
    RB_CHECK(contains(server.lastRejectReason(), "command_source_lease_token_mismatch"));

    RB_CHECK(server.parseMessage(
        R"({"seq":2,"mode":"JointTarget","source_id":"policy_runner","session_id":"policy-session","lease_token":"good-token","q_target_deg":[1,2,3,4,5,6]})",
        now + 3,
        &out
    ));
    RB_CHECK(out.lease.command_requires_lease);
    RB_CHECK(out.lease.command_has_lease);
    return true;
}

bool testDefaultOffAndEmergencyOverride() {
    rb_servo::NetworkConfig permissive_network;
    rb_servo::CommandBuffer permissive_buffer;
    rb_servo::CommandServer permissive(permissive_network, &permissive_buffer);
    rb_servo::DualArmCommand out;
    const uint64_t now = rb_servo::nowSteadyNs();

    RB_CHECK(!permissive_network.command_source_enforce_lease);
    RB_CHECK(permissive.parseMessage(
        R"({"seq":1,"mode":"AcquireLease","source_id":"rb_gui","session_id":"gui-session"})",
        now,
        &out
    ));
    RB_CHECK(out.lease.active);
    RB_CHECK(!out.lease.enforce_lease);

    RB_CHECK(permissive.parseMessage(
        R"({"seq":1,"mode":"JointTarget","source_id":"policy_runner","session_id":"policy-session","q_target_deg":[1,2,3,4,5,6]})",
        now + 1,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(out.left.has_joint_target);
    RB_CHECK(!out.lease.enforce_lease);
    RB_CHECK(!out.lease.command_has_lease);

    rb_servo::NetworkConfig enforcing_network;
    enforcing_network.command_source_enforce_lease = true;
    rb_servo::CommandBuffer enforcing_buffer;
    rb_servo::CommandServer enforcing(enforcing_network, &enforcing_buffer);
    RB_CHECK(enforcing.parseMessage(
        R"({"seq":1,"mode":"AcquireLease","source_id":"rb_gui","session_id":"gui-session"})",
        now,
        &out
    ));
    RB_CHECK(enforcing.parseMessage(
        R"({"seq":1,"mode":"EmergencyStop","source_id":"policy_runner","session_id":"policy-session"})",
        now + 1,
        &out
    ));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::EmergencyStop);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::EmergencyStop);
    RB_CHECK(!out.lease.command_requires_lease);
    return true;
}

bool testReleaseLeaseAllowsImmediateTakeover() {
    rb_servo::NetworkConfig network;
    network.command_source_enforce_lease = true;
    network.command_source_lease_timeout_sec = 60.0;
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    rb_servo::DualArmCommand out;
    const uint64_t now = rb_servo::nowSteadyNs();

    RB_CHECK(server.parseMessage(
        R"({"seq":1,"mode":"AcquireLease","source_id":"policy_runner","session_id":"old-session"})",
        now,
        &out
    ));
    RB_CHECK(out.lease.active);
    const std::string policy_token = out.lease.lease_token;

    // A foreign/stale session cannot release the live owner's lease.
    RB_CHECK(!server.parseMessage(
        R"({"seq":1,"mode":"ReleaseLease","source_id":"policy_runner","session_id":"new-session"})",
        now + 1,
        &out
    ));
    RB_CHECK(contains(server.lastRejectReason(), "command_source_lease_release_denied"));

    // A release with the wrong token is rejected.
    RB_CHECK(!server.parseMessage(
        R"({"seq":2,"mode":"ReleaseLease","source_id":"policy_runner","session_id":"old-session","lease_token":"wrong-token"})",
        now + 2,
        &out
    ));
    RB_CHECK(contains(server.lastRejectReason(), "command_source_lease_token_mismatch"));

    // The owning session releases (with its token); lease becomes inactive.
    const std::string release = std::string(
        R"({"seq":3,"mode":"ReleaseLease","source_id":"policy_runner","session_id":"old-session","lease_token":")"
    ) + policy_token + R"("})";
    RB_CHECK(server.parseMessage(release, now + 3, &out));
    RB_CHECK(!out.lease.active);

    // A new session acquires immediately — no 60 s stale-lease wait.
    RB_CHECK(server.parseMessage(
        R"({"seq":1,"mode":"AcquireLease","source_id":"policy_runner","session_id":"new-session"})",
        now + 4,
        &out
    ));
    RB_CHECK(out.lease.active);
    RB_CHECK(out.lease.session_id == "new-session");

    // Releasing when no lease is active is an accepted no-op.
    rb_servo::CommandServer idle_server(network, &buffer);
    RB_CHECK(idle_server.parseMessage(
        R"({"seq":1,"mode":"ReleaseLease","source_id":"policy_runner","session_id":"any-session"})",
        now,
        &out
    ));
    RB_CHECK(!out.lease.active);
    return true;
}

bool testLeaseAdminUpdatesBufferReadbackWithoutDisplacingMotion() {
    // Regression: lease-admin packets skip CommandBuffer::setCommand so they do
    // not displace the buffered motion command, but the lease grant must still
    // reach the published state (lease readback = snapshot.command.lease).
    // Without updateLease an acquiring client polls forever for a grant it
    // already has (deadlock: lazy lease -> no motion command -> no readback).
    rb_servo::CommandBuffer buffer;
    const uint64_t now = rb_servo::nowSteadyNs();

    rb_servo::DualArmCommand motion;
    motion.seq = 7;
    motion.host_time_ns = 0;  // never times out in this test
    motion.left.mode = rb_servo::ControlMode::TcpPoseTarget;
    motion.left.has_tcp_target = true;
    motion.left.tcp_target_stand = {0.4, 0.0, 0.3, 0.0, 0.0, 0.0};
    motion.right.mode = rb_servo::ControlMode::Hold;
    buffer.setCommand(motion);

    rb_servo::CommandSourceLeaseState lease;
    lease.active = true;
    lease.source_id = "policy_runner";
    lease.session_id = "policy-session";
    lease.lease_token = "tok";
    buffer.updateLease(lease, now);

    rb_servo::DualArmCommand out = buffer.latestOrHold(now);
    RB_CHECK(out.seq == 7);  // motion command not displaced
    RB_CHECK(out.left.mode == rb_servo::ControlMode::TcpPoseTarget);
    RB_CHECK(out.lease.active);
    RB_CHECK(out.lease.source_id == "policy_runner");
    RB_CHECK(out.lease.lease_token == "tok");

    // Acquire at startup (empty buffer): the readback must still surface via a
    // synthesized non-expiring Hold.
    rb_servo::CommandBuffer empty;
    empty.updateLease(lease, now);
    out = empty.latestOrHold(now);
    RB_CHECK(out.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(out.right.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(out.lease.active);
    RB_CHECK(out.lease.session_id == "policy-session");

    // Regression (teleop re-engage after idle): the buffer still holds the
    // LAST streaming command, already expired. Writing the lease onto that
    // expired carrier hides it — latestOrHold falls back to a fresh empty-lease
    // Hold and the acquiring client's readback never sees the grant. The lease
    // must ride a synthesized non-expiring Hold instead.
    rb_servo::CommandBuffer idle;
    rb_servo::DualArmCommand stale;
    stale.seq = 9;
    stale.host_time_ns = now > 10'000'000'000ull ? now - 10'000'000'000ull : 1;  // ~10s ago
    stale.left.mode = rb_servo::ControlMode::TcpPoseTarget;
    stale.right.mode = rb_servo::ControlMode::TcpPoseTarget;
    stale.left.has_tcp_target = true;
    stale.right.has_tcp_target = true;
    stale.left.tcp_target_stand = {0.4, 0.0, 0.3, 0.0, 0.0, 0.0};
    stale.right.tcp_target_stand = {0.4, 0.0, 0.3, 0.0, 0.0, 0.0};
    stale.left.timeout_sec = 0.3;
    stale.right.timeout_sec = 0.3;
    idle.setCommand(stale);
    idle.updateLease(lease, now);
    out = idle.latestOrHold(now);
    RB_CHECK(out.left.mode == rb_servo::ControlMode::Hold);  // stale motion not revived
    RB_CHECK(out.lease.active);
    RB_CHECK(out.lease.source_id == "policy_runner");
    RB_CHECK(out.lease.session_id == "policy-session");
    return true;
}

}  // namespace

int main() {
    if (!testAcquireLeaseAndExpiration()) return 1;
    if (!testWrongTokenRejected()) return 1;
    if (!testDefaultOffAndEmergencyOverride()) return 1;
    if (!testReleaseLeaseAllowsImmediateTakeover()) return 1;
    if (!testLeaseAdminUpdatesBufferReadbackWithoutDisplacingMotion()) return 1;
    std::cout << "command source lease tests passed\n";
    return 0;
}
