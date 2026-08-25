from __future__ import annotations

import unittest

from rb_servo_gui.realtime_health import realtime_health_html


class RealtimeHealthTest(unittest.TestCase):
    def test_renders_499_hz_misses_feedback_phase_and_inference(self) -> None:
        state = {
            "realtime_timing": {
                "window_sec": 10.0,
                "servo": {
                    "target_rate_hz": 500.0,
                    "observed_rate_hz": 499.03,
                    "send_rate_hz": 499.02,
                    "period_ms": {"last": 2.001, "p95": 2.008, "max": 4.01},
                    "jitter_ms": {"last": 0.001, "p95": 0.008, "max": 2.01},
                    "send_duration_us": {"last": 211.0, "p95": 287.0, "max": 422.0},
                    "wake_latency_us": {"last": 21.0, "p95": 47.0, "max": 2023.0},
                    "deadline_miss_count": 1,
                    "catch_up_count": 1,
                },
                "feedback": {
                    "left": {
                        "frame_rate_hz": 500.0,
                        "fresh_rate_hz": 500.0,
                        "freshness_reliable": True,
                        "held_count": 0,
                        "age_us": {"p95": 812.0},
                        "jitter_ms": {"p95": 0.041},
                        "phase_us": {"p95": 733.0},
                    },
                    "right": {
                        "frame_rate_hz": 499.0,
                        "fresh_rate_hz": 499.0,
                        "freshness_reliable": False,
                        "held_count": 2,
                        "age_us": {"p95": 1210.0},
                        "jitter_ms": {"p95": 0.052},
                        "phase_us": {"p95": 1311.0},
                    },
                },
            }
        }
        inference = {
            "inference_timing": {
                "inference_rate_hz": 2.13,
                "inference_ms": {"last": 171.0, "p95": 184.2, "max": 201.0},
                "queue_wait_ms": {"p95": 2.4},
                "ready_wait_ms": {"last": 18.0},
                "period_ms": {"p95": 481.0},
                "stream_stall_count": 0,
            }
        }

        html = realtime_health_html(state, inference)

        self.assertIn("SERVO_J", html)
        self.assertIn("499.03 / 500 Hz", html)
        self.assertIn("deadline miss 1", html)
        self.assertIn("catch-up 1", html)
        self.assertIn("dispatch p95 287 µs", html)
        self.assertIn("L 500.00 Hz · R 499.00 Hz", html)
        self.assertIn("jitter p95 0.041 ms", html)
        self.assertIn("phase p95 733 µs", html)
        self.assertIn("controller fresh unverified", html)
        self.assertIn("inference p95 184.20 ms", html)
        self.assertIn("499 Hz는 숨기지 않고 소수점으로 표시", html)

    def test_missing_optional_schema_is_explicit(self) -> None:
        html = realtime_health_html({}, None, stale=True)

        self.assertEqual(html.count("telemetry unavailable"), 3)
        self.assertIn("서버 realtime_timing 집계를 기다리는 중", html)
        self.assertIn("policy chunk inference_timing을 기다리는 중", html)

    def test_policy_runner_rolling_inference_schema_is_consumed(self) -> None:
        inference = {
            "inference_timing": {
                "inference_period_ms": 400.0,
                "ready_wait_ms": 7.5,
                "stall_count": 2,
                "rolling": {
                    "queue_wait_ms": {"p95_ms": 3.0, "max_ms": 4.0},
                    "inference_latency_ms": {"p95_ms": 155.0, "max_ms": 180.0},
                    "inference_period_ms": {"p95_ms": 420.0, "max_ms": 450.0},
                },
                "inference_period_jitter": {"p95_ms": 12.0, "max_ms": 20.0},
            }
        }

        html = realtime_health_html({}, inference)

        self.assertIn("2.50 Hz", html)
        self.assertIn("inference p95 155.00 ms", html)
        self.assertIn("queue p95 3.00 ms", html)
        self.assertIn("ready→activate 7.50 ms", html)
        self.assertIn("period p95 420.00 ms", html)
        self.assertIn("jitter p95 12.00 ms", html)
        self.assertIn("stall 2", html)


if __name__ == "__main__":
    unittest.main()
