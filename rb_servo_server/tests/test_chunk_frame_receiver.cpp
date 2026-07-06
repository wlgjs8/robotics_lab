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

static std::string makePacket(int seq, int horizon, bool with_right) {
  std::string left = "[";
  for (int i = 0; i < horizon; ++i) {
    left += "[" + std::to_string(0.01 * i) + ",0.0,0.2,0.0,0.0,0.0,1.0," +
            std::to_string(20.0 + i) + "]";
    if (i + 1 < horizon) left += ",";
  }
  left += "]";
  std::string right = with_right ? left : "null";
  return std::string("{\"schema\":\"robotics_lab.chunk_overlay.v2\",") +
         "\"schema_version\":\"robotics_lab.chunk_overlay.v2\"," +
         "\"host_time_ns\":123456789," +
         "\"seq\":" + std::to_string(seq) + "," +
         "\"policy_dt_sec\":0.0334," +
         "\"horizon\":" + std::to_string(horizon) + "," +
         "\"left\":" + left + ",\"right\":" + right + "}";
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
  }

  // -- Test 3: oversize horizon clamps to kMaxSteps. ---------------------------
  std::printf("Test 3: horizon clamp\n");
  {
    const std::string pkt = makePacket(1, ChunkFrameReceiver::kMaxSteps + 8, false);
    ChunkFrameReceiver::Frame frame;
    check(ChunkFrameReceiver::parsePacket(pkt.data(), pkt.size(), &frame), "oversize packet parses");
    check(frame.left.count == ChunkFrameReceiver::kMaxSteps, "steps clamped to kMaxSteps");
  }

  // -- Test 4: live loopback receive + receiver_seq dedup. ---------------------
  std::printf("Test 4: loopback receive\n");
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

  // -- Test 5: empty bind = disabled but start() succeeds. ---------------------
  std::printf("Test 5: disabled receiver\n");
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
