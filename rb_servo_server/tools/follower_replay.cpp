// follower_replay.cpp — offline CartesianChunkFollower replay + limit sweep.
//
// Feeds a RECORDED absolute stand-frame knot sequence (one pose7 per policy step,
// extracted from a rollout JSONL by tools/extract_knots.py) through the real
// CartesianChunkFollower and reports the Ruckig segment duration distribution.
//
// Why: the follower solves one segment per policy step with minimum_duration=dt,
// consumes only the first dt, then re-solves toward the next knot. duration/dt is
// therefore how much of each planned motion actually executes -- and on hardware
// it sits at ~3.1 at speed 1.0 (trembling) vs ~1.2 at 0.75 (smooth), pinned to a
// single dominant value, which says a fixed limit profile is binding rather than
// the task. This tool identifies WHICH limit, deterministically and without
// touching the robot, so the hardware only has to confirm one candidate.
//
// Usage:
//   follower_replay <knots.txt> [--dt SEC] [--execute N] [--runway N]
//                   [--lin-v V --lin-a A --lin-j J] [--ang-v V --ang-a A --ang-j J]
//                   [--beta B | --beta-lin B --beta-ang B] [--corner-ang R] [--corner-scale S]
//                   [--sweep]
//
// --beta sets both axis classes (legacy); --beta-lin/--beta-ang override per class. The split
// exists because translation and rotation sit at very different fractions of their acceleration
// limits (31% vs 95% on the right arm at SPEED_SCALE 1.0) -- see control::GuardConfig.

#include "rb_servo/control/cartesian_chunk_follower.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <fstream>
#include <string>
#include <vector>

using rb_servo::control::CartesianChunkFollower;
using rb_servo::control::CartesianChunkFollowerConfig;
using rb_servo::control::ChunkFrame;
using rb_servo::Pose6D;

namespace {

struct Options {
  std::string path;
  double dt = 1.0 / 30.0;
  int execute = 5;
  int runway = 4;
  double lin_v = 0.45, lin_a = 5.0, lin_j = 300.0;
  double ang_v = 0.90, ang_a = 9.0, ang_j = 540.0;
  double beta_lin = 1.0;
  double beta_ang = 1.0;
  double corner_ang = 5e-4;
  double corner_lin = 3e-4;
  double corner_scale = 0.25;
  bool sweep = false;
};

std::vector<Pose6D> loadKnots(const std::string& path) {
  std::vector<Pose6D> out;
  std::ifstream in(path);
  double x, y, z, qx, qy, qz, qw;
  while (in >> x >> y >> z >> qx >> qy >> qz >> qw) {
    const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (!(n > 1e-9)) continue;
    Eigen::Quaterniond q(qw / n, qx / n, qy / n, qz / n);
    out.push_back(rb_servo::math::poseFromSe3(
        pinocchio::SE3(q.toRotationMatrix(), Eigen::Vector3d(x, y, z))));
  }
  return out;
}

struct Result {
  double p50 = 0, p90 = 0, p99 = 0, max = 0;
  double over15 = 0, over20 = 0, converged = 0, corner = 0;
  double proj_p95_mm = 0;
  int segments = 0;
};

double pct(std::vector<double>& v, double q) {
  if (v.empty()) return 0.0;
  const std::size_t i = static_cast<std::size_t>(
      std::min<double>(v.size() - 1, std::max(0.0, q * (v.size() - 1))));
  std::nth_element(v.begin(), v.begin() + static_cast<long>(i), v.end());
  return v[i];
}

// Replays the knot stream the way the producer publishes it: a fresh frame every
// `execute` steps carrying execute+runway rows, submitted to an already-active
// follower (preempt), exactly like the boundary stitch mode on hardware.
Result replay(const std::vector<Pose6D>& knots, const Options& o) {
  CartesianChunkFollowerConfig cfg;
  cfg.lin = rb_servo::control::AxisLimit{o.lin_v, o.lin_a, o.lin_j};
  cfg.ang = rb_servo::control::AxisLimit{o.ang_v, o.ang_a, o.ang_j};
  cfg.window = {/*L*/ 0, /*C*/ o.execute, /*R*/ o.runway, /*smooth*/ 1};
  cfg.guard.af_damping_beta_lin = o.beta_lin;
  cfg.guard.af_damping_beta_ang = o.beta_ang;
  cfg.guard.corner_deadband_lin_m = o.corner_lin;
  cfg.guard.corner_deadband_ang_rad = o.corner_ang;
  cfg.guard.corner_velocity_scale = o.corner_scale;

  CartesianChunkFollower f(cfg);
  std::vector<double> ratios;
  Result r;
  int converged = 0, corner = 0;
  std::vector<double> proj;

  const int rows = o.execute + o.runway;
  const int tick_per_seg = std::max(1, static_cast<int>(std::lround(o.dt / 0.002)));
  int last_segments = -1;

  for (std::size_t start = 0; start + static_cast<std::size_t>(rows) < knots.size();
       start += static_cast<std::size_t>(o.execute)) {
    ChunkFrame frame;
    frame.policy_dt = o.dt;
    frame.wire_seq = static_cast<std::uint64_t>(start);
    frame.recv_seq = static_cast<std::uint64_t>(start);
    for (int i = 0; i < rows; ++i) frame.pose.push_back(knots[start + static_cast<std::size_t>(i)]);
    frame.grip.assign(frame.pose.size(), 20.0);
    f.submitFrame(frame, knots[start]);

    for (int s = 0; s < o.execute; ++s) {
      for (int t = 0; t < tick_per_seg; ++t) f.tick(0.002);
      const auto& d = f.diag();
      if (d.segments != last_segments) {   // one sample per solved segment
        last_segments = d.segments;
        if (d.last_solve.duration > 0.0) {
          ratios.push_back(d.last_solve.duration / o.dt);
          if (d.last_solve.converged) ++converged;
          if (d.last_solve.corner) ++corner;
          proj.push_back(d.projection_error_m * 1000.0);
          ++r.segments;
        }
      }
    }
  }
  if (ratios.empty()) return r;
  std::vector<double> tmp = ratios;
  r.p50 = pct(tmp, 0.50); tmp = ratios;
  r.p90 = pct(tmp, 0.90); tmp = ratios;
  r.p99 = pct(tmp, 0.99);
  r.max = *std::max_element(ratios.begin(), ratios.end());
  r.over15 = 100.0 * static_cast<double>(std::count_if(ratios.begin(), ratios.end(),
                 [](double v) { return v > 1.5; })) / static_cast<double>(ratios.size());
  r.over20 = 100.0 * static_cast<double>(std::count_if(ratios.begin(), ratios.end(),
                 [](double v) { return v > 2.0; })) / static_cast<double>(ratios.size());
  r.converged = 100.0 * converged / r.segments;
  r.corner = 100.0 * corner / r.segments;
  r.proj_p95_mm = pct(proj, 0.95);
  return r;
}

void printRow(const char* label, const Result& r) {
  std::printf("%-34s segs=%-5d p50=%6.3f p90=%6.3f p99=%6.3f max=%6.2f  >1.5=%5.1f%% >2=%5.1f%%  conv=%5.1f%% corner=%5.1f%% projP95=%6.2fmm\n",
              label, r.segments, r.p50, r.p90, r.p99, r.max, r.over15, r.over20,
              r.converged, r.corner, r.proj_p95_mm);
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  if (argc < 2) { std::fprintf(stderr, "usage: %s <knots.txt> [opts]\n", argv[0]); return 2; }
  o.path = argv[1];
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    auto num = [&](double& dst) { if (i + 1 < argc) dst = std::atof(argv[++i]); };
    auto inum = [&](int& dst) { if (i + 1 < argc) dst = std::atoi(argv[++i]); };
    if (a == "--dt") num(o.dt);
    else if (a == "--execute") inum(o.execute);
    else if (a == "--runway") inum(o.runway);
    else if (a == "--lin-v") num(o.lin_v);
    else if (a == "--lin-a") num(o.lin_a);
    else if (a == "--lin-j") num(o.lin_j);
    else if (a == "--ang-v") num(o.ang_v);
    else if (a == "--ang-a") num(o.ang_a);
    else if (a == "--ang-j") num(o.ang_j);
    else if (a == "--beta") { double b = o.beta_lin; num(b); o.beta_lin = b; o.beta_ang = b; }
    else if (a == "--beta-lin") num(o.beta_lin);
    else if (a == "--beta-ang") num(o.beta_ang);
    else if (a == "--corner-ang") num(o.corner_ang);
    else if (a == "--corner-scale") num(o.corner_scale);
    else if (a == "--sweep") o.sweep = true;
  }
  const std::vector<Pose6D> knots = loadKnots(o.path);
  if (knots.size() < 20) { std::fprintf(stderr, "need >=20 knots, got %zu\n", knots.size()); return 2; }
  std::printf("knots=%zu  dt=%.4fs  execute=%d runway=%d\n", knots.size(), o.dt, o.execute, o.runway);

  if (!o.sweep) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "lin a=%.1f j=%.0f | ang a=%.1f j=%.0f | beta lin=%.2f ang=%.2f",
                  o.lin_a, o.lin_j, o.ang_a, o.ang_j, o.beta_lin, o.beta_ang);
    printRow(buf, replay(knots, o));
    return 0;
  }

  // One-factor-at-a-time from the tracked baseline: whichever single change moves
  // p50 is the binding limit.
  printRow("BASELINE (tracked stack_real)", replay(knots, o));
  struct Variant { const char* label; Options (*apply)(Options); };
  const std::vector<std::pair<const char*, std::function<void(Options&)>>> variants = {
      {"lin_a 5->10",        [](Options& x) { x.lin_a = 10.0; }},
      {"lin_a 5->20",        [](Options& x) { x.lin_a = 20.0; }},
      {"lin_j 300->1000",    [](Options& x) { x.lin_j = 1000.0; }},
      {"lin_j 300->3000",    [](Options& x) { x.lin_j = 3000.0; }},
      {"ang_a 9->20",        [](Options& x) { x.ang_a = 20.0; }},
      {"ang_a 9->50",        [](Options& x) { x.ang_a = 50.0; }},
      {"ang_j 540->2000",    [](Options& x) { x.ang_j = 2000.0; }},
      {"ang_j 540->6000",    [](Options& x) { x.ang_j = 6000.0; }},
      {"ang_v 0.9->3.0",     [](Options& x) { x.ang_v = 3.0; }},
      {"lin_v 0.45->1.5",    [](Options& x) { x.lin_v = 1.5; }},
      {"beta both ->0.5",    [](Options& x) { x.beta_lin = 0.5; x.beta_ang = 0.5; }},
      {"beta both ->0(min)", [](Options& x) { x.beta_lin = 1e-9; x.beta_ang = 1e-9; }},
      // The proposed split: rotation is the saturated class, translation carries descent
      // authority and must stay honest.
      {"beta ang ->0.5",     [](Options& x) { x.beta_ang = 0.5; }},
      {"beta ang ->0.25",    [](Options& x) { x.beta_ang = 0.25; }},
      {"beta ang ->0.1",     [](Options& x) { x.beta_ang = 0.1; }},
      {"beta lin ->0.25",    [](Options& x) { x.beta_lin = 0.25; }},
      {"corner_scale->1.0",  [](Options& x) { x.corner_scale = 1.0; }},
  };
  for (const auto& [label, apply] : variants) {
    Options v = o;
    apply(v);
    printRow(label, replay(knots, v));
  }
  return 0;
}
