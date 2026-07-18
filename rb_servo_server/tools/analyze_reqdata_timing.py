#!/usr/bin/env python3
"""Correlate rbpodo request_data() call timing with a passive packet capture.

The application CSV supplies request_data() call start/return timestamps. tshark
supplies host capture timestamps for the outbound ``reqdata`` TCP payload and
the first inbound CobotData SystemState frame packet. All correlation uses the
host system clock; steady-clock timestamps remain in the source CSV for local
duration validation.
"""

from __future__ import annotations

import argparse
import csv
import shutil
import subprocess
import sys
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Iterable, Sequence


DATA_PORT = 5001
REQDATA_PAYLOAD = b"reqdata"


class AnalysisError(Exception):
    """Raised when input cannot support trustworthy event correlation."""


@dataclass(frozen=True)
class PacketEvent:
    timestamp_system_ns: int
    arm: str
    kind: str
    payload_length: int


@dataclass(frozen=True)
class RequestDataCall:
    tick: int
    arm: str
    exchange_sequence: int
    source: str
    call_start_steady_ns: int
    call_start_system_ns: int
    call_return_steady_ns: int
    call_return_system_ns: int
    call_duration_us: float
    backend_read_total_duration_us: float


OUTPUT_COLUMNS = (
    "tick",
    "arm",
    "exchange_sequence",
    "timing_source",
    "request_data_call_start_steady_ns",
    "request_data_call_start_system_ns",
    "reqdata_tx_packet_system_ns",
    "controller_response_first_rx_packet_system_ns",
    "request_data_call_return_steady_ns",
    "request_data_call_return_system_ns",
    "call_start_to_reqdata_tx_us",
    "reqdata_tx_to_response_first_rx_us",
    "response_first_rx_to_request_data_return_us",
    "system_clock_call_boundary_duration_us",
    "request_data_call_duration_us",
    "system_minus_steady_call_duration_us",
    "backend_read_total_duration_us",
    "backend_read_outside_request_data_us",
    "dominant_phase",
    "classification",
    "slow",
)


def epoch_seconds_to_ns(value: str) -> int:
    try:
        return int(Decimal(value) * Decimal(1_000_000_000))
    except (InvalidOperation, ValueError) as exc:
        raise AnalysisError(f"invalid frame.time_epoch value: {value!r}") from exc


def decode_tcp_payload(value: str) -> bytes:
    compact = value.replace(":", "").strip()
    if not compact:
        return b""
    try:
        return bytes.fromhex(compact)
    except ValueError as exc:
        raise AnalysisError("tshark returned a non-hex tcp.payload field") from exc


def parse_tshark_lines(
    lines: Iterable[str], controller_ips: dict[str, str]
) -> list[PacketEvent]:
    arm_by_ip = {ip: arm for arm, ip in controller_ips.items()}
    events: list[PacketEvent] = []
    for line_number, raw_line in enumerate(lines, start=1):
        line = raw_line.rstrip("\r\n")
        if not line:
            continue
        fields = line.split("\t")
        if len(fields) != 6:
            raise AnalysisError(
                f"unexpected tshark field count at line {line_number}: {len(fields)}"
            )
        epoch, src_ip, src_port_text, dst_ip, dst_port_text, payload_text = fields
        try:
            src_port = int(src_port_text)
            dst_port = int(dst_port_text)
        except ValueError as exc:
            raise AnalysisError(
                f"invalid TCP port at tshark line {line_number}"
            ) from exc
        payload = decode_tcp_payload(payload_text)
        timestamp_ns = epoch_seconds_to_ns(epoch)

        if dst_ip in arm_by_ip and dst_port == DATA_PORT and payload.startswith(REQDATA_PAYLOAD):
            events.append(
                PacketEvent(timestamp_ns, arm_by_ip[dst_ip], "reqdata_tx", len(payload))
            )
            continue
        # CobotData SystemState framing is '$', size_lo, size_hi, type=0x03.
        # The timestamp intentionally means first response-frame packet arrival,
        # not completion of the potentially multi-segment state frame.
        if (
            src_ip in arm_by_ip
            and src_port == DATA_PORT
            and len(payload) >= 4
            and payload[0] == ord("$")
            and payload[3] == 0x03
        ):
            events.append(
                PacketEvent(timestamp_ns, arm_by_ip[src_ip], "response_first_rx", len(payload))
            )
    return sorted(events, key=lambda event: event.timestamp_system_ns)


def read_packet_events(pcap: Path, controller_ips: dict[str, str]) -> list[PacketEvent]:
    tshark = shutil.which("tshark")
    if tshark is None:
        raise AnalysisError("tshark is required to analyze the packet capture")
    command = [
        tshark,
        "-r",
        str(pcap),
        "-Y",
        "tcp.port == 5001 && tcp.len > 0",
        "-T",
        "fields",
        "-E",
        "separator=\\t",
        "-E",
        "quote=n",
        "-E",
        "occurrence=f",
        "-e",
        "frame.time_epoch",
        "-e",
        "ip.src",
        "-e",
        "tcp.srcport",
        "-e",
        "ip.dst",
        "-e",
        "tcp.dstport",
        "-e",
        "tcp.payload",
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        detail = completed.stderr.strip() or "unknown tshark error"
        raise AnalysisError(f"tshark failed: {detail}")
    return parse_tshark_lines(completed.stdout.splitlines(), controller_ips)


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no", ""}:
        return False
    raise AnalysisError(f"invalid boolean field: {value!r}")


def read_calls(servo_log: Path) -> list[RequestDataCall]:
    with servo_log.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = set(reader.fieldnames or ())
        required = {"tick"}
        for arm in ("left", "right"):
            required.update(
                {
                    f"{arm}_reqdata_timing_available",
                    f"{arm}_reqdata_exchange_sequence",
                    f"{arm}_reqdata_timing_source",
                    f"{arm}_reqdata_call_start_steady_ns",
                    f"{arm}_reqdata_call_start_system_ns",
                    f"{arm}_reqdata_call_return_steady_ns",
                    f"{arm}_reqdata_call_return_system_ns",
                    f"{arm}_reqdata_call_duration_us",
                    f"{arm}_worker_loop_read_duration_us",
                }
            )
        missing = sorted(required - fieldnames)
        if missing:
            raise AnalysisError(
                "servo CSV does not contain reqdata timing columns: " + ", ".join(missing)
            )

        calls: list[RequestDataCall] = []
        for row_number, row in enumerate(reader, start=2):
            try:
                tick = int(row["tick"])
                for arm in ("left", "right"):
                    if not parse_bool(row[f"{arm}_reqdata_timing_available"]):
                        continue
                    call = RequestDataCall(
                        tick=tick,
                        arm=arm,
                        exchange_sequence=int(row[f"{arm}_reqdata_exchange_sequence"]),
                        source=row[f"{arm}_reqdata_timing_source"],
                        call_start_steady_ns=int(row[f"{arm}_reqdata_call_start_steady_ns"]),
                        call_start_system_ns=int(row[f"{arm}_reqdata_call_start_system_ns"]),
                        call_return_steady_ns=int(row[f"{arm}_reqdata_call_return_steady_ns"]),
                        call_return_system_ns=int(row[f"{arm}_reqdata_call_return_system_ns"]),
                        call_duration_us=float(row[f"{arm}_reqdata_call_duration_us"]),
                        backend_read_total_duration_us=float(
                            row[f"{arm}_worker_loop_read_duration_us"]
                        ),
                    )
                    if (
                        call.call_start_system_ns <= 0
                        or call.call_return_system_ns < call.call_start_system_ns
                        or call.call_return_steady_ns < call.call_start_steady_ns
                    ):
                        raise AnalysisError("invalid request_data call timestamp ordering")
                    calls.append(call)
            except (KeyError, TypeError, ValueError, AnalysisError) as exc:
                raise AnalysisError(f"invalid servo CSV row {row_number}: {exc}") from exc
    return sorted(calls, key=lambda call: call.call_start_system_ns)


def diff_us(newer_ns: int | None, older_ns: int | None) -> float | None:
    if newer_ns is None or older_ns is None:
        return None
    return (newer_ns - older_ns) / 1000.0


def dominant_phase(phases: Sequence[tuple[str, float | None]]) -> str:
    available = [(name, value) for name, value in phases if value is not None]
    if not available:
        return "unknown"
    return max(available, key=lambda item: abs(item[1]))[0]


def correlate_events(
    calls: Sequence[RequestDataCall],
    packet_events: Sequence[PacketEvent],
    tolerance_us: float,
    slow_threshold_us: float,
) -> tuple[list[dict[str, object]], set[int]]:
    tolerance_ns = int(tolerance_us * 1000.0)
    consumed: set[int] = set()
    rows: list[dict[str, object]] = []

    for call in calls:
        tx_index: int | None = None
        for index, event in enumerate(packet_events):
            if index in consumed or event.arm != call.arm or event.kind != "reqdata_tx":
                continue
            if event.timestamp_system_ns < call.call_start_system_ns - tolerance_ns:
                continue
            if event.timestamp_system_ns > call.call_return_system_ns + tolerance_ns:
                break
            tx_index = index
            break

        tx_ns = packet_events[tx_index].timestamp_system_ns if tx_index is not None else None
        if tx_index is not None:
            consumed.add(tx_index)

        rx_index: int | None = None
        if tx_ns is not None:
            for index, event in enumerate(packet_events):
                if index in consumed or event.arm != call.arm or event.kind != "response_first_rx":
                    continue
                if event.timestamp_system_ns < tx_ns:
                    continue
                if event.timestamp_system_ns > call.call_return_system_ns + tolerance_ns:
                    break
                rx_index = index
                break
        rx_ns = packet_events[rx_index].timestamp_system_ns if rx_index is not None else None
        if rx_index is not None:
            consumed.add(rx_index)

        start_to_tx = diff_us(tx_ns, call.call_start_system_ns)
        tx_to_rx = diff_us(rx_ns, tx_ns)
        rx_to_return = diff_us(call.call_return_system_ns, rx_ns)
        system_call_duration = diff_us(
            call.call_return_system_ns, call.call_start_system_ns
        )
        system_minus_steady = (
            system_call_duration - call.call_duration_us
            if system_call_duration is not None
            else None
        )
        if tx_ns is None:
            classification = "missing_reqdata_tx_packet"
        elif rx_ns is None:
            classification = "missing_response_first_rx_packet"
        elif (
            min(start_to_tx or 0.0, tx_to_rx or 0.0, rx_to_return or 0.0)
            < -tolerance_us
            or abs(system_minus_steady or 0.0) > tolerance_us
        ):
            classification = "clock_or_correlation_mismatch"
        else:
            classification = "complete"
        phase = dominant_phase(
            (
                ("call_start_to_reqdata_tx", start_to_tx),
                ("reqdata_tx_to_response_first_rx", tx_to_rx),
                ("response_first_rx_to_request_data_return", rx_to_return),
            )
        )
        rows.append(
            {
                "tick": call.tick,
                "arm": call.arm,
                "exchange_sequence": call.exchange_sequence,
                "timing_source": call.source,
                "request_data_call_start_steady_ns": call.call_start_steady_ns,
                "request_data_call_start_system_ns": call.call_start_system_ns,
                "reqdata_tx_packet_system_ns": tx_ns,
                "controller_response_first_rx_packet_system_ns": rx_ns,
                "request_data_call_return_steady_ns": call.call_return_steady_ns,
                "request_data_call_return_system_ns": call.call_return_system_ns,
                "call_start_to_reqdata_tx_us": start_to_tx,
                "reqdata_tx_to_response_first_rx_us": tx_to_rx,
                "response_first_rx_to_request_data_return_us": rx_to_return,
                "system_clock_call_boundary_duration_us": system_call_duration,
                "request_data_call_duration_us": call.call_duration_us,
                "system_minus_steady_call_duration_us": system_minus_steady,
                "backend_read_total_duration_us": call.backend_read_total_duration_us,
                "backend_read_outside_request_data_us": (
                    call.backend_read_total_duration_us - call.call_duration_us
                ),
                "dominant_phase": phase,
                "classification": classification,
                "slow": call.call_duration_us >= slow_threshold_us,
            }
        )
    return rows, consumed


def percentile(values: Sequence[float], percentile_value: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * percentile_value
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def format_stat(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.3f}"


def print_summary(
    rows: Sequence[dict[str, object]],
    packet_event_count: int,
    consumed_count: int,
) -> None:
    print(f"calls={len(rows)} packet_events={packet_event_count} unmatched_packet_events={packet_event_count - consumed_count}")
    for arm in ("left", "right"):
        arm_rows = [row for row in rows if row["arm"] == arm]
        if not arm_rows:
            continue
        complete = [row for row in arm_rows if row["classification"] == "complete"]
        slow = sum(bool(row["slow"]) for row in arm_rows)
        print(f"{arm}: calls={len(arm_rows)} complete={len(complete)} slow={slow}")
        for column in (
            "request_data_call_duration_us",
            "call_start_to_reqdata_tx_us",
            "reqdata_tx_to_response_first_rx_us",
            "response_first_rx_to_request_data_return_us",
            "system_minus_steady_call_duration_us",
            "backend_read_outside_request_data_us",
        ):
            values = [
                float(row[column])
                for row in complete
                if row[column] is not None
            ]
            print(
                f"  {column}: p50={format_stat(percentile(values, 0.50))} "
                f"p95={format_stat(percentile(values, 0.95))} "
                f"max={format_stat(max(values) if values else None)}"
            )


def write_rows(output_csv: Path, rows: Sequence[dict[str, object]]) -> None:
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--servo-log", required=True, type=Path)
    parser.add_argument("--pcap", required=True, type=Path)
    parser.add_argument("--left-ip", required=True)
    parser.add_argument("--right-ip", required=True)
    parser.add_argument("--output-csv", required=True, type=Path)
    parser.add_argument(
        "--correlation-tolerance-us",
        type=float,
        default=1_000.0,
        help="Allowed capture/application clock-boundary mismatch (default: 1000 us)",
    )
    parser.add_argument(
        "--slow-threshold-us",
        type=float,
        default=5_000.0,
        help="request_data duration classified as slow (default: 5000 us)",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.correlation_tolerance_us < 0 or args.slow_threshold_us < 0:
            raise AnalysisError("timing thresholds must be non-negative")
        calls = read_calls(args.servo_log)
        if not calls:
            raise AnalysisError("servo CSV contains no direct request_data timing samples")
        events = read_packet_events(
            args.pcap, {"left": args.left_ip, "right": args.right_ip}
        )
        rows, consumed = correlate_events(
            calls,
            events,
            args.correlation_tolerance_us,
            args.slow_threshold_us,
        )
        write_rows(args.output_csv, rows)
        print_summary(rows, len(events), len(consumed))
        print(f"wrote {args.output_csv}")
        return 0
    except (AnalysisError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
