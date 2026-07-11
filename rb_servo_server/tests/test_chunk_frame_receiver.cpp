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

static std::string makePacket(int seq, int horizon, bool with_right, bool with_delta = false) {
  std::string left = "[";
  std::string left_delta = "[";
  for (int i = 0; i < horizon; ++i) {
    left += "[" + std::to_string(0.01 * i) + ",0.0,0.2,0.0,0.0,0.0,1.0," +
            std::to_string(20.0 + i) + "]";
    left_delta += "[" + std::to_string(0.001 * i) + ",0.002,0.003,0.004,0.005,0.006," +
                  std::to_string(20.0 + i) + "]";
    if (i + 1 < horizon) left += ",";
    if (i + 1 < horizon) left_delta += ",";
  }
  left += "]";
  left_delta += "]";
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
  out += "}";
  return out;
}

static std::string withDiagnostics(std::string packet) {
  packet.pop_back();
  packet +=
      ",\"chunk_execute_steps\":12,\"chunk_overlay_runway_steps\":4"
      ",\"inference_timing\":{\"seq\":91,\"queue_wait_ms\":1.25,"
      "\"inference_latency_ms\":77.5,\"ready_wait_ms\":3.5,"
      "\"inference_period_ms\":401.0,\"inference_period_jitter_ms\":6.0,"
      "\"stall_count\":2}"
      ",\"camera_diagnostics\":{\"bundle_seq\":301,\"bundle_age_ms\":12.5,"
      "\"max_skew_ms\":4.25,\"left_frame_number\":1001,"
      "\"right_frame_number\":1002,\"left_frame_age_ms\":8.0,"
      "\"right_frame_age_ms\":9.0,\"left_focus_score\":44.5,"
      "\"right_focus_score\":45.5}}";
  return packet;
}

int main() {
  // -- Test 1: parsePacket accepts the producer's overlay wire format. --------
  std::printf("Test 1: parsePacket\n");
  {
    const std::string pkt = withDiagnostics(makePacket(7, 16, /*with_right=*/false));
    ChunkFrameReceiver::Frame frame;
    check(ChunkFrameReceiver::parsePacket(pkt.data(), pkt.size(), &frame), "valid packet parses");
    check(frame.seq == 7, "producer seq parsed");
    check(frame.has_left && !frame.has_right, "left present, right null");
    check(!frame.has_left_delta && !frame.has_right_delta, "old packet has no deltas");
    check(frame.left.count == 16, "16 steps");
    check(frame.left.step[3][0] == 0.03, "step x values land");
    check(frame.left.step[3][7] == 23.0, "grip values land");
    check(frame.policy_dt_sec > 0.033 && frame.policy_dt_sec < 0.034, "policy_dt parsed");
    check(frame.diagnostics.execute_steps == 12, "execute steps diagnostic parsed");
    check(frame.diagnostics.runway_steps == 4, "runway steps diagnostic parsed");
    check(frame.diagnostics.inference_seq == 91, "inference seq diagnostic parsed");
    check(frame.diagnostics.inference_latency_ms == 77.5, "inference latency diagnostic parsed");
    check(frame.diagnostics.inference_stall_count == 2, "inference stall diagnostic parsed");
    check(frame.diagnostics.camera_bundle_seq == 301, "camera bundle diagnostic parsed");
    check(frame.diagnostics.camera_right_focus_score == 45.5, "camera focus diagnostic parsed");
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
    std::string optional_bad = makePacket(2, 1, false);
    optional_bad.pop_back();
    optional_bad += ",\"chunk_execute_steps\":\"bad\",\"inference_timing\":{\"inference_latency_ms\":\"bad\",\"stall_count\":-1},\"camera_diagnostics\":[]}";
    check(ChunkFrameReceiver::parsePacket(optional_bad.data(), optional_bad.size(), &frame),
          "malformed optional diagnostics do not reject motion frame");
    check(frame.diagnostics.execute_steps == 0 &&
          frame.diagnostics.inference_latency_ms == 0.0 &&
          frame.diagnostics.inference_stall_count == 0,
          "malformed optional diagnostics remain zero");
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

  // -- Test 4: oversize horizon clamps to kMaxSteps. ---------------------------
  std::printf("Test 4: horizon clamp\n");
  {
    const std::string pkt = makePacket(1, ChunkFrameReceiver::kMaxSteps + 8, false, true);
    ChunkFrameReceiver::Frame frame;
    check(ChunkFrameReceiver::parsePacket(pkt.data(), pkt.size(), &frame), "oversize packet parses");
    check(frame.left.count == ChunkFrameReceiver::kMaxSteps, "steps clamped to kMaxSteps");
    check(frame.left_delta.count == ChunkFrameReceiver::kMaxSteps, "delta steps clamped to kMaxSteps");
  }

  // -- Test 5: live loopback receive + receiver_seq dedup. ---------------------
  std::printf("Test 5: loopback receive\n");
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
    check(frame.interarrival_sec > 0.0, "receiver interarrival stamped");

    ::close(sender);
    receiver.stop();
    check(true, "receiver stops cleanly");
  }

  // -- Test 6: empty bind = disabled but start() succeeds. ---------------------
  std::printf("Test 6: disabled receiver\n");
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
