// test_chunk_frame_receiver.cpp — wire-format parse (the producer's
// chunk-overlay packet) + live loopback receive + dedup seq consistency.

#include "rb_servo/network/chunk_frame_receiver.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

using rb_servo::ChunkFrameReceiver;

static int g_failures = 0;
static void check(bool ok, const char* name) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

static std::string makePacket(
    int seq,
    int horizon,
    bool with_right,
    bool with_delta = false,
    bool with_grip_horizon = false
) {
  std::string left = "[";
  std::string left_delta = "[";
  std::string grip_horizon = "[";
  for (int i = 0; i < horizon; ++i) {
    left += "[" + std::to_string(0.01 * i) + ",0.0,0.2,0.0,0.0,0.0,1.0," +
            std::to_string(20.0 + i) + "]";
    left_delta += "[" + std::to_string(0.001 * i) + ",0.002,0.003,0.004,0.005,0.006," +
                  std::to_string(20.0 + i) + "]";
    grip_horizon += std::to_string(40.0 - i);
    if (i + 1 < horizon) left += ",";
    if (i + 1 < horizon) left_delta += ",";
    if (i + 1 < horizon) grip_horizon += ",";
  }
  left += "]";
  left_delta += "]";
  grip_horizon += "]";
  std::string right = with_right ? left : "null";
  std::string out = std::string("{\"schema\":\"robotics_lab.chunk_overlay.v2\",") +
         "\"schema_version\":\"robotics_lab.chunk_overlay.v2\"," +
         "\"host_time_ns\":123456789," +
         "\"seq\":" + std::to_string(seq) + "," +
         "\"policy_dt_sec\":0.0334," +
         "\"horizon\":" + std::to_string(horizon) + "," +
         "\"left\":" + left + ",\"right\":" + right;
  if (with_delta) {
    out += ",\"left_delta\":" + left_delta;
    if (with_right) out += ",\"right_delta\":" + left_delta;
  }
  if (with_grip_horizon) {
    out += ",\"left_grip_horizon\":" + grip_horizon;
    if (with_right) out += ",\"right_grip_horizon\":" + grip_horizon;
  }
  out += "}";
  return out;
}

int main() {
  // -- Test 1: parsePacket accepts the producer's overlay wire format. --------
  std::printf("Test 1: parsePacket\n");
  {
    const std::string pkt = makePacket(7, 16, /*with_right=*/false);
    ChunkFrameReceiver::Frame frame;
    check(ChunkFrameReceiver::parsePacket(pkt.data(), pkt.size(), &frame), "valid packet parses");
    check(frame.seq == 7, "producer seq parsed");
    check(frame.has_left && !frame.has_right, "left present, right null");
    check(!frame.has_left_delta && !frame.has_right_delta, "old packet has no deltas");
    check(!frame.has_left_grip_horizon && !frame.has_right_grip_horizon,
          "old packet has no grip horizon");
    check(frame.left.count == 16, "16 steps");
    check(frame.left.step[3][0] == 0.03, "step x values land");
    check(frame.left.step[3][7] == 23.0, "grip values land");
    check(frame.policy_dt_sec > 0.033 && frame.policy_dt_sec < 0.034, "policy_dt parsed");
  }

  // -- Test 2: malformed / wrong-schema packets are rejected. ------------------
  std::printf("Test 2: rejects\n");
  {
    ChunkFrameReceiver::Frame frame;
    const std::string bad1 = "not json at all";
    check(!ChunkFrameReceiver::parsePacket(bad1.data(), bad1.size(), &frame), "garbage rejected");
    const std::string bad2 = "{\"schema_version\":\"other.schema\",\"seq\":1,\"policy_dt_sec\":0.033,\"left\":[[0,0,0,0,0,0,1,0]]}";
    check(!ChunkFrameReceiver::parsePacket(bad2.data(), bad2.size(), &frame), "wrong schema rejected");
    const std::string bad3 = "{\"schema_version\":\"robotics_lab.chunk_overlay.v2\",\"seq\":1,\"policy_dt_sec\":0.033,\"left\":null,\"right\":null}";
    check(!ChunkFrameReceiver::parsePacket(bad3.data(), bad3.size(), &frame), "both arms null rejected");
    const std::string bad4 = "{\"schema_version\":\"robotics_lab.chunk_overlay.v2\",\"seq\":1,\"policy_dt_sec\":0.033,\"left\":[[0,0,0,0,0,0,0,0]]}";
    check(!ChunkFrameReceiver::parsePacket(bad4.data(), bad4.size(), &frame), "degenerate quaternion rejected");
    const std::string bad5 = "{\"schema_version\":\"robotics_lab.chunk_overlay.v2\",\"seq\":1,\"policy_dt_sec\":0.033,\"left\":[[0,0,0,0,0,0,1,0]],\"left_delta\":[[0,0,0,0,0,\"bad\",0]]}";
    check(!ChunkFrameReceiver::parsePacket(bad5.data(), bad5.size(), &frame), "malformed delta row rejected");
    const std::string bad6 = "{\"schema_version\":\"robotics_lab.chunk_overlay.v2\",\"seq\":1,\"policy_dt_sec\":0.033,\"left\":[[0,0,0,0,0,0,1,0]],\"left_grip_horizon\":[40,\"bad\",20]}";
    check(!ChunkFrameReceiver::parsePacket(bad6.data(), bad6.size(), &frame), "malformed grip horizon rejected");
  }

  // -- Test 3: optional delta rows parse. -------------------------------------
  std::printf("Test 3: delta rows\n");
  {
    const std::string pkt = makePacket(8, 4, true, true);
    ChunkFrameReceiver::Frame frame;
    check(ChunkFrameReceiver::parsePacket(pkt.data(), pkt.size(), &frame), "packet with deltas parses");
    check(frame.has_left_delta && frame.has_right_delta, "delta presence flags set");
    check(frame.left_delta.count == 4, "left delta count parsed");
    check(frame.right_delta.count == 4, "right delta count parsed");
    check(frame.left_delta.step[3][0] == 0.003, "left delta dx values land");
    check(frame.left_delta.step[3][6] == 23.0, "left delta grip values land");
  }

  // -- Test 4: optional full grip horizon parses. ----------------------------
  std::printf("Test 4: grip horizon rows\n");
  {
    const std::string pkt = makePacket(9, 24, true, true, true);
    ChunkFrameReceiver::Frame frame;
    check(ChunkFrameReceiver::parsePacket(pkt.data(), pkt.size(), &frame),
          "packet with grip horizon parses");
    check(frame.has_left_grip_horizon && frame.has_right_grip_horizon,
          "grip horizon presence flags set");
    check(frame.left_grip_horizon.count == 24, "left grip horizon count parsed");
    check(frame.right_grip_horizon.count == 24, "right grip horizon count parsed");
    check(frame.left_grip_horizon.value[0] == 40.0, "left grip horizon first value lands");
    check(frame.left_grip_horizon.value[16] == 24.0, "late grip horizon value lands");
  }

  // -- Test 5: oversize horizon clamps to kMaxSteps. ---------------------------
  std::printf("Test 5: horizon clamp\n");
  {
    const std::string pkt =
        makePacket(1, ChunkFrameReceiver::kMaxSteps + 8, false, true, true);
    ChunkFrameReceiver::Frame frame;
    check(ChunkFrameReceiver::parsePacket(pkt.data(), pkt.size(), &frame), "oversize packet parses");
    check(frame.left.count == ChunkFrameReceiver::kMaxSteps, "steps clamped to kMaxSteps");
    check(frame.left_delta.count == ChunkFrameReceiver::kMaxSteps, "delta steps clamped to kMaxSteps");
    check(frame.left_grip_horizon.count == ChunkFrameReceiver::kMaxGripHorizon,
          "grip horizon clamped to kMaxGripHorizon");
  }

  // -- Test 6: live loopback receive + receiver_seq dedup. ---------------------
  std::printf("Test 6: loopback receive\n");
  {
    const int port = 57263;
    ChunkFrameReceiver receiver("udp://127.0.0.1:" + std::to_string(port));
    check(receiver.start(), "receiver starts");

    const int sender = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    ::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    const std::string pkt1 = makePacket(55, 16, true);
    ::sendto(sender, pkt1.data(), pkt1.size(), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));

    ChunkFrameReceiver::Frame frame;
    bool got = false;
    for (int i = 0; i < 200 && !got; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      got = receiver.latestSeq() > 0 && receiver.copyLatest(&frame);
    }
    check(got, "frame received over loopback");
    check(frame.seq == 55, "producer seq remains wire seq");
    check(frame.receiver_seq == 1, "receiver_seq assigned");
    check(frame.has_left && frame.has_right, "both arms parsed");
    check(frame.recv_steady_sec > 0.0, "receive time stamped");

    const std::string pkt2 = makePacket(56, 16, true);
    ::sendto(sender, pkt2.data(), pkt2.size(), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    got = false;
    for (int i = 0; i < 200 && !got; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      got = receiver.copyLatest(&frame) && frame.receiver_seq == 2;
    }
    check(got, "second frame supersedes (receiver_seq=2)");
    check(frame.seq == 56, "producer seq follows independently");

    ::close(sender);
    receiver.stop();
    check(true, "receiver stops cleanly");
  }

  // -- Test 7: empty bind = disabled but start() succeeds. ---------------------
  std::printf("Test 7: disabled receiver\n");
  {
    ChunkFrameReceiver receiver("");
    check(receiver.start(), "empty bind starts as no-op");
    check(receiver.latestSeq() == 0, "no frames ever");
    receiver.stop();
  }

  std::printf("\n=== %s (%d failure%s) ===\n",
              g_failures == 0 ? "ALL TESTS PASSED" : "TEST FAILURES",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
