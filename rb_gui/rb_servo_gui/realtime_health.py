from __future__ import annotations

import math
from html import escape
from typing import Any, Mapping


_TONES = {
    "ok": ("#e0f5e9", "#148a4e", "#22aa63"),
    "warn": ("#fcf3de", "#8a6410", "#e1a01e"),
    "bad": ("#fae4e4", "#b34646", "#dc4646"),
    "info": ("#dbe7fe", "#2563eb", "#2563eb"),
    "muted": ("#eef0f4", "#52606d", "#9aa4b2"),
}


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, int | float):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _metric(block: Mapping[str, Any], name: str, stat: str = "last") -> float | None:
    nested = _mapping(block.get(name))
    value = _number(nested.get(stat))
    if value is not None:
        return value
    aliases = {
        "last": (name, f"{name}_last"),
        "p50": (f"{name}_p50",),
        "p95": (f"{name}_p95",),
        "p99": (f"{name}_p99",),
        "max": (f"{name}_max",),
    }
    for key in aliases.get(stat, ()):
        value = _number(block.get(key))
        if value is not None:
            return value
    return None


def _first_number(block: Mapping[str, Any], *keys: str) -> float | None:
    for key in keys:
        value = _number(block.get(key))
        if value is not None:
            return value
    return None


def _rolling_metric(block: Mapping[str, Any], name: str, stat: str) -> float | None:
    value = _metric(block, name, stat)
    if value is not None:
        return value
    summary = _mapping(_mapping(block.get("rolling")).get(name))
    return _number(summary.get(f"{stat}_ms"))


def _count(block: Mapping[str, Any], *keys: str) -> int:
    value = _first_number(block, *keys)
    return max(0, int(value)) if value is not None else 0


def _fmt(value: float | None, digits: int = 2, suffix: str = "") -> str:
    return "—" if value is None else f"{value:.{digits}f}{suffix}"


def _rate_tone(rate_hz: float | None, target_hz: float | None, misses: int) -> str:
    if rate_hz is None or target_hz is None or target_hz <= 0.0:
        return "muted"
    error_hz = abs(rate_hz - target_hz)
    if misses > 0 or error_hz > 2.0:
        return "bad"
    if error_hz > 0.5:
        return "warn"
    return "ok"


def _card(title: str, headline: str, details: str, tone: str) -> str:
    bg, fg, dot = _TONES.get(tone, _TONES["muted"])
    return (
        f'<div style="flex:1 1 19em;min-width:17em;background:{bg};color:{fg};'
        'border-radius:0.55em;padding:0.58em 0.72em;margin:0.18em;'
        'font-family:system-ui,-apple-system,sans-serif;">'
        f'<div style="display:flex;align-items:center;gap:0.45em;font-weight:700;">'
        f'<span style="width:0.55em;height:0.55em;border-radius:50%;background:{dot};"></span>'
        f'<span style="font-size:12px;opacity:0.72;">{escape(title)}</span>'
        f'<span style="font-size:14px;font-variant-numeric:tabular-nums;">{escape(headline)}</span></div>'
        f'<div style="font-size:11px;margin-top:0.28em;line-height:1.45;'
        f'font-variant-numeric:tabular-nums;white-space:pre-line;">{escape(details)}</div></div>'
    )


def _feedback_summary(block: Mapping[str, Any], label: str) -> tuple[str, str, float | None, int]:
    rate = _first_number(block, "frame_rate_hz", "observed_rate_hz", "received_rate_hz")
    fresh_rate = _first_number(block, "fresh_rate_hz", "controller_fresh_rate_hz")
    age_us = _metric(block, "age_us", "p95")
    phase_us = _metric(block, "phase_us", "p95")
    jitter_ms = _metric(block, "jitter_ms", "p95")
    held = _count(block, "held_count", "duplicate_count")
    reliable = block.get("freshness_reliable")
    rate_text = _fmt(rate, 2, " Hz")
    fresh_text = _fmt(fresh_rate, 2, " Hz") if reliable is True else "unverified"
    details = (
        f"{label} frame {rate_text} · controller fresh {fresh_text} · "
        f"age p95 {_fmt(age_us / 1000.0 if age_us is not None else None, 3, ' ms')} · "
        f"jitter p95 {_fmt(jitter_ms, 3, ' ms')} · phase p95 {_fmt(phase_us, 0, ' µs')} · held {held}"
    )
    return rate_text, details, rate, held


def realtime_health_html(
    state_raw: Mapping[str, Any] | None,
    inference_raw: Mapping[str, Any] | None = None,
    *,
    stale: bool = False,
) -> str:
    """Render server-owned 500 Hz aggregates without reconstructing them at GUI rate."""

    timing = _mapping(_mapping(state_raw).get("realtime_timing"))
    servo = _mapping(timing.get("servo"))
    feedback = _mapping(timing.get("feedback"))
    target_hz = _first_number(servo, "target_rate_hz", "target_hz")
    rate_hz = _first_number(servo, "observed_rate_hz", "rate_hz")
    send_hz = _first_number(servo, "send_rate_hz")
    jitter_p95 = _metric(servo, "jitter_ms", "p95")
    period_p95 = _metric(servo, "period_ms", "p95")
    send_p95 = _metric(servo, "send_duration_us", "p95")
    wake_p95 = _metric(servo, "wake_latency_us", "p95")
    misses = _count(servo, "deadline_miss_count", "deadline_misses")
    catchups = _count(servo, "catch_up_count", "catchup_count")
    servo_available = bool(servo) and rate_hz is not None
    servo_tone = "bad" if stale and servo_available else _rate_tone(rate_hz, target_hz, misses)
    servo_head = "telemetry unavailable" if not servo_available else f"{rate_hz:.2f} / {_fmt(target_hz, 0)} Hz"
    servo_detail = (
        f"send {_fmt(send_hz, 2, ' Hz')} · period p95 {_fmt(period_p95, 3, ' ms')} · "
        f"jitter p95 {_fmt(jitter_p95, 3, ' ms')} · dispatch p95 {_fmt(send_p95, 0, ' µs')} · "
        f"wake p95 {_fmt(wake_p95, 0, ' µs')} · "
        f"deadline miss {misses} · catch-up {catchups}"
    ) if servo_available else "서버 realtime_timing 집계를 기다리는 중"

    left = _mapping(feedback.get("left"))
    right = _mapping(feedback.get("right"))
    left_rate_text, left_detail, left_rate, left_held = _feedback_summary(left, "L")
    right_rate_text, right_detail, right_rate, right_held = _feedback_summary(right, "R")
    feedback_available = bool(left) or bool(right)
    target_feedback = target_hz or 500.0
    rates = [value for value in (left_rate, right_rate) if value is not None]
    worst_rate = min(rates) if rates else None
    feedback_tone = _rate_tone(worst_rate, target_feedback, left_held + right_held)
    if stale and feedback_available:
        feedback_tone = "bad"
    feedback_head = (
        f"L {left_rate_text} · R {right_rate_text}" if feedback_available else "telemetry unavailable"
    )
    feedback_detail = (
        left_detail + "\n" + right_detail if feedback_available else "feedback frame 집계를 기다리는 중"
    )

    inference = _mapping(_mapping(inference_raw).get("inference_timing"))
    infer_rate = _first_number(inference, "rate_hz", "inference_rate_hz")
    current_period = _first_number(inference, "inference_period_ms", "period_ms")
    if infer_rate is None and current_period is not None and current_period > 0.0:
        infer_rate = 1000.0 / current_period
    infer_p95 = _rolling_metric(inference, "inference_ms", "p95")
    if infer_p95 is None:
        infer_p95 = _rolling_metric(inference, "latency_ms", "p95")
    if infer_p95 is None:
        infer_p95 = _rolling_metric(inference, "inference_latency_ms", "p95")
    queue_p95 = _rolling_metric(inference, "queue_wait_ms", "p95")
    ready_wait = _metric(inference, "ready_wait_ms", "last")
    period_p95_infer = _rolling_metric(inference, "period_ms", "p95")
    if period_p95_infer is None:
        period_p95_infer = _rolling_metric(inference, "inference_period_ms", "p95")
    jitter_p95_infer = _rolling_metric(inference, "jitter_ms", "p95")
    if jitter_p95_infer is None:
        jitter_p95_infer = _number(_mapping(inference.get("inference_period_jitter")).get("p95_ms"))
    stalls = _count(inference, "stall_count", "stream_stall_count")
    infer_available = bool(inference)
    infer_tone = "warn" if stalls > 0 else "info" if infer_available else "muted"
    infer_head = _fmt(infer_rate, 2, " Hz") if infer_available else "telemetry unavailable"
    infer_detail = (
        f"inference p95 {_fmt(infer_p95, 2, ' ms')} · queue p95 {_fmt(queue_p95, 2, ' ms')} · "
        f"ready→activate {_fmt(ready_wait, 2, ' ms')} · period p95 {_fmt(period_p95_infer, 2, ' ms')} · "
        f"jitter p95 {_fmt(jitter_p95_infer, 2, ' ms')} · stall {stalls}"
    ) if infer_available else "policy chunk inference_timing을 기다리는 중"

    cards = [
        _card("SERVO_J", servo_head, servo_detail, servo_tone),
        _card("FEEDBACK / F·T", feedback_head, feedback_detail, feedback_tone),
        _card("MODEL", infer_head, infer_detail, infer_tone),
    ]
    window_sec = _first_number(timing, "window_sec")
    footer = (
        f'<div style="width:100%;font-size:10px;color:#7b8492;margin:0.15em 0.45em;">'
        f"producer rolling window: {_fmt(window_sec, 1, ' s')} · phase는 2 ms scheduled tick 기준 · "
        "499 Hz는 숨기지 않고 소수점으로 표시</div>"
    )
    return '<div style="display:flex;flex-wrap:wrap;margin:0.25em -0.18em;">' + "".join(cards) + footer + "</div>"
