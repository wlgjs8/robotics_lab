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

}  // namespace

int main() {
    if (!testAcquireLeaseAndExpiration()) return 1;
    if (!testWrongTokenRejected()) return 1;
    if (!testDefaultOffAndEmergencyOverride()) return 1;
    std::cout << "command source lease tests passed\n";
    return 0;
}
