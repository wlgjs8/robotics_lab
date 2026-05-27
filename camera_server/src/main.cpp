#include "camera_server/camera/camera_manager.hpp"
#include "camera_server/config/config.hpp"
#include "camera_server/health/health_monitor.hpp"
#include "camera_server/publish/metadata_publisher.hpp"
#include "camera_server/recording/recorder.hpp"
#include "camera_server/shm/shared_memory_ring.hpp"
#include "camera_server/sync/frame_synchronizer.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_running{true};
void on_signal(int) { g_running = false; }

void usage() {
  std::cerr << "usage: camera_server --config config/triple_realsense.yaml [--simulate] [--run-seconds N]\n";
}
}  // namespace

int main(int argc, char** argv) {
  std::string config_path;
  bool simulate = false;
  int run_seconds = 0;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) config_path = argv[++i];
    else if (arg == "--simulate") simulate = true;
    else if (arg == "--run-seconds" && i + 1 < argc) run_seconds = std::stoi(argv[++i]);
    else if (arg == "--help" || arg == "-h") { usage(); return 0; }
    else { usage(); return 2; }
  }
  if (config_path.empty()) {
    const char* env = std::getenv("CAMERA_SERVER_CONFIG");
    if (env) config_path = env;
  }
  if (config_path.empty()) {
    usage();
    return 2;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  try {
    camera_server::AppConfig cfg = camera_server::load_config(config_path);
    if (simulate) cfg.server.simulate_cameras = true;
    std::cerr << "[CAM] config=" << config_path << " mode=" << cfg.server.mode
              << " simulate=" << (cfg.server.simulate_cameras ? 1 : 0) << '\n';

    camera_server::SharedMemoryRingBuffer shm;
    shm.create(cfg);
    camera_server::MetadataPublisher publisher(cfg.metadata);
    publisher.start();
    camera_server::FrameSynchronizer synchronizer(cfg);
    camera_server::Recorder recorder(cfg.recording);
    recorder.start();
    camera_server::CameraManager manager(cfg, shm, publisher, synchronizer, recorder);
    camera_server::HealthMonitor health(cfg.health, publisher, [&manager] { return manager.snapshot(); });
    manager.start();
    health.start();

    const auto start = std::chrono::steady_clock::now();
    while (g_running) {
      if (run_seconds > 0 && std::chrono::steady_clock::now() - start >= std::chrono::seconds(run_seconds)) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    health.stop();
    manager.stop();
    recorder.stop();
    publisher.stop();
    std::cerr << "[CAM] shutdown complete\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[CAM] fatal: " << e.what() << '\n';
    return 1;
  }
}
