#!/usr/bin/env python3
"""Build and run firmware-loop simulations and independently decode RF frames.

All dependencies are in the standard library plus a host C compiler. Reports
contain attempted/delivered logs and the RF frequencies reconstructed from the
production driver's register writes. No connected device is accessed.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import time
import types
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = [
    "diag_no_data", "diag_uart_errors", "temp_cold", "temp_hot",
    "no_altitude", "gps_priority", "gps_late_reacquire", "no_inputs", "usb_waiting", "gps_no_usb", "gps_usb", "serial_only",
    "simultaneous", "gps_during_serial_frame", "gps_loss", "gps_silent",
    "gps_reacquire", "moving", "midnight", "fractional", "near_slot_before",
    "near_slot_after", "altitude_after_slot", "time_only", "location_only", "malformed", "noisy_uart",
    "usb_reconnect", "usb_backpressure", "i2c_init_failure", "i2c_start_failure",
    "i2c_symbol_failure", "i2c_disable_failure", "long_holdover",
]


def decode_frame(tones):
    """Hard-decision decoder, independent of the C encoder.

    Invert the bit-reversal permutation, then walk the noiseless convolutional
    trellis from zero state. Unpack the recovered 50-bit WSPR type-1 payload.
    This tests the emitted message, not just whether its tone count is 162.
    """
    assert len(tones) == 162
    symbols = [round((hz - 14097100) / (375 / 256)) for _, hz in tones]
    assert all(0 <= s <= 3 for s in symbols)
    sync = [
        1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,
        0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,
        0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,
        0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,1,0,0,
        0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,
        1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,1,0,0,0,0,
        0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,
        0,0,0,1,1,0,0,0,
    ]
    assert [symbol & 1 for symbol in symbols] == sync, "invalid WSPR synchronization sequence"
    assert all(abs(hz - (14097100 + s * 375 / 256)) < 0.12
               for (_, hz), s in zip(tones, symbols))
    destinations = [int(f"{i:08b}"[::-1], 2) for i in range(256)]
    coded = [symbols[i] >> 1 for i in destinations if i < 162]
    state, bits = 0, []
    for index in range(0, 162, 2):
        candidates = []
        for bit in (0, 1):
            candidate = ((state << 1) | bit) & 0xFFFFFFFF
            parity = [(candidate & mask).bit_count() % 2 for mask in (0xF2D05351, 0xE4613C47)]
            if parity == coded[index:index + 2]:
                candidates.append((bit, candidate))
        assert len(candidates) == 1, f"invalid convolutional code at bit {index // 2}"
        bit, state = candidates[0]
        bits.append(bit)
    assert bits[50:] == [0] * 31, "nonzero convolutional tail"
    packed = int("".join(map(str, bits[:50])), 2)
    call, location_power = packed >> 22, packed & ((1 << 22) - 1)
    suffix = []
    for _ in range(3):
        call, digit = divmod(call, 27)
        suffix.append("ABCDEFGHIJKLMNOPQRSTUVWXYZ "[digit])
    call, number = divmod(call, 10)
    first, second = divmod(call, 36)
    alphabet = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ "
    callsign = (alphabet[first] + alphabet[second] + str(number) + "".join(reversed(suffix))).strip()
    location, power = divmod(location_power, 128)
    longitude = 179 - location // 180
    latitude = location % 180
    grid = chr(65 + longitude // 10) + chr(65 + latitude // 10) + str(longitude % 10) + str(latitude % 10)
    return {"callsign": callsign, "grid": grid, "payload": location, "power_dbm": power - 64}


def check_report(report):
    name = report["scenario"]
    frames = report["frames"]
    assert report["outputs_ever_enabled"] & 0xFE == 0, "unused clock output enabled"
    assert report["unused_outputs_ever_powered"] == 0, "unused clock output powered up"
    assert report["uart_overruns"] == 0, "GPS UART overflowed"
    assert report["uart_read"] == report["uart_bytes"], "GPS monitoring stopped early"
    logs = sorted(report["logs"], key=lambda item: item["at_us"])
    texts = [item["text"] for item in logs]
    if name.startswith("diag_"):
        diagnostics = [t for t in texts if t.startswith("GPS DIAG:")]
        assert len(diagnostics) == 6, diagnostics
        summary = diagnostics[0]
        assert "sentences=0 checksum_failures=0" in summary
        if name == "diag_no_data":
            assert "bytes=0" in summary and "last_byte_ms=-1" in summary
            assert "framing=0 parity=0 breaks=0 overruns=0" in summary
            assert "none" in diagnostics[1]
        else:
            assert "bytes=3" in summary
            assert "framing=3 parity=3 breaks=3 overruns=3" in summary
            assert "FF 55 7F" in diagnostics[1]
        assert all(any(label in t for t in diagnostics) for label in
                   ("GPS_RX", "GPS_LSWITCH", "GPS_RESET", "GPS_V_BCKP"))
        assert any("GPS_LSWITCH GP2 digital=0 function=5 sio_dir=out" in t for t in diagnostics)
    if name == "gps_priority":
        diagnostics = [log for log in logs if log["text"].startswith("GPS DIAG:")]
        assert len(diagnostics) == 6
        assert all(20_000_000 <= log["at_us"] < 20_010_000 for log in diagnostics)
    expected_starts = [11, 131]
    expected_grids = ["JN58", "JN58"]
    if name in {"no_inputs", "usb_waiting", "time_only", "location_only", "malformed",
                "i2c_init_failure", "i2c_start_failure", "no_altitude", "diag_no_data", "diag_uart_errors"}:
        expected_starts = expected_grids = []
    elif name == "serial_only":
        expected_grids = [report["fallback_grid"]] * 2
    elif name == "gps_during_serial_frame":
        expected_starts, expected_grids = [1, 131], [report["fallback_grid"], "JN58"]
    elif name in {"moving", "gps_priority"}:
        expected_starts = list(range(11, 852, 120))
        expected_grids = ["JN58"] * 4 + (["FN41"] * 4 if name == "moving" else ["JN58"] * 4)
    elif name == "gps_late_reacquire":
        expected_starts = [11, 131, 251, 371, 611, 731, 851, 971]
        expected_grids = ["JN58"] * 4 + ["FN41"] * 4
    elif name in {"near_slot_after", "altitude_after_slot"}:
        expected_starts, expected_grids = [131], ["JN58"]
    elif name in {"i2c_symbol_failure", "i2c_disable_failure"}:
        expected_starts, expected_grids = [11], ["JN58"]
    elif name in {"long_holdover", "gps_loss", "gps_silent", "temp_cold", "temp_hot"}:
        expected_starts = [11, 131, 251, 371]
        expected_grids = ["JN58"] * len(expected_starts)
    assert len(frames) == len(expected_starts), f"expected {len(expected_starts)} frames, got {len(frames)}"
    assert report["outputs_ever_enabled"] == (1 if frames else 0)
    max_boundary_error = 0
    decoded = []
    for frame_index, (frame, start, grid) in enumerate(zip(frames, expected_starts, expected_grids)):
        # UART start capture, polling and I2C wire time introduce sub-ms error.
        assert abs(frame["start_us"] - start * 1_000_000) <= 3000, frame["start_us"]
        if name == "i2c_symbol_failure":
            assert 20_000_000 <= frame["end_us"] < 21_000_000
            assert 1 < len(frame["tones"]) < 162
            continue
        decoded.append(decode_frame(frame["tones"]))
        gps_index = frame_index - (1 if name == "gps_during_serial_frame" else 0)
        phase = gps_index % 4 if name != "serial_only" and gps_index >= 0 else 0
        call = report[["callsign", "callsign_fine", "callsign_alt", "callsign_temp"][phase]].upper()
        assert decoded[-1]["callsign"] == call, decoded[-1]
        assert decoded[-1]["power_dbm"] == report["power_dbm"]
        if phase == 0:
            assert decoded[-1]["grid"] == grid, decoded[-1]
        elif phase == 1:
            # Munich: 11°31' E, 48°7.038' N => offsets (136, 21).
            assert decoded[-1]["payload"] == (0 if grid == "FN41" else 136 * 180 + 21), decoded[-1]
        elif phase == 2:
            assert decoded[-1]["payload"] == (457 if grid == "FN41" else 123), decoded[-1]
        else:
            # Munich fractional fine-cell offsets => extra indices (7, 1).
            extra_index = 0 if grid == "FN41" else 7 * 15 + 1
            temperature = -80 if name == "temp_cold" else 47 if name == "temp_hot" else 27 if gps_index < 4 else -31
            assert decoded[-1]["payload"] == extra_index * 128 + temperature + 80, decoded[-1]
            assert decoded[-1]["payload"] < 32400
        for index, (at, _) in enumerate(frame["tones"]):
            error = abs(at - frame["start_us"] - index * 8192000000 // 12000)
            max_boundary_error = max(max_boundary_error, error)
            assert error <= 600, f"symbol {index} missed deadline by {error} us"
        if name == "i2c_disable_failure":
            assert frame["end_us"] == 0 and report["rf_enabled"]
        else:
            assert abs(frame["end_us"] - frame["start_us"] - 110_592_000) <= 600
    # One discarded ADC read plus 16 averaged samples per new GPS snapshot.
    if not name.startswith("i2c_"):
        snapshots = 0 if name == "serial_only" else sum(
            (i - (1 if name == "gps_during_serial_frame" else 0)) >= 0 and
            (i - (1 if name == "gps_during_serial_frame" else 0)) % 4 == 0
            for i in range(len(frames)))
        assert report["adc_reads"] == snapshots * 17, report["adc_reads"]
    if name != "i2c_disable_failure":
        assert not report["rf_enabled"], "RF left enabled"
        assert report["output_disable_mask"] == 0xFF, "idle state did not disable all outputs"
    else:
        assert report["output_disable_mask"] == 0xFE
    if name.startswith("i2c_"):
        assert report["fault_count"] > 0
        assert any("beacon halted" in text for text in texts)
        assert any(("RF STATE: UNKNOWN" if name == "i2c_disable_failure" else "RF STATE: NOT TRANSMITTING")
                   in text for text in texts)
    else:
        assert not any("WSPR ERROR" in text for text in texts), texts
    if name in {"gps_no_usb", "no_inputs", "long_holdover", "usb_backpressure"}:
        assert not any(log["delivered"] for log in logs)
    if name == "usb_reconnect":
        snapshots = [log for log in logs if "WSPR STATUS" in log["text"] and log["delivered"]]
        assert len(snapshots) == 2
        assert 150_000_000 <= snapshots[1]["at_us"] < 150_001_000
        assert "RF=TRANSMITTING" in snapshots[1]["text"]
    if name == "noisy_uart":
        assert report["checksum_failures"] >= 1
    if name in {"gps_loss", "gps_silent", "gps_reacquire", "long_holdover"}:
        assert any(log["at_us"] >= 20_000_000 and "GPS STATE" in log["text"]
                   and "UTC=unavailable" in log["text"] for log in logs)
    if name == "gps_during_serial_frame":
        assert any(5_000_000 <= log["at_us"] < 6_000_000 and "UTC=valid" in log["text"] for log in logs)
    if name == "serial_only":
        assert report["serial_read"] == 7
    if name in {"gps_usb", "gps_no_usb", "fractional", "midnight"}:
        # Repeated TXT and fluctuating nonzero SNR must not flood the terminal.
        assert len(logs) < 30, f"too many logs: {len(logs)}"
        assert sum("GPS MODULE" in text for text in texts) == 1
    if name == "usb_waiting":
        assert len(logs) <= 7, "unchanged idle state is noisy"
    return {"scenario": name, "frames": len(frames), "decoded": decoded,
            "max_symbol_error_us": max_boundary_error, "log_lines": len(logs),
            "uart_bytes": report["uart_read"], "simulated_seconds": report["duration_us"] / 1e6}


def check_trigger_script():
    # Import the actual script without installing pyserial or opening any port.
    spec = importlib.util.spec_from_file_location("wspr_trigger", ROOT / "wspr_serial_trigger.py")
    trigger = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {"serial": types.ModuleType("serial")}), \
            patch.object(sys, "dont_write_bytecode", True):
        spec.loader.exec_module(trigger)
    origin = datetime(2026, 9, 5, tzinfo=timezone.utc).timestamp()
    tested = 0
    for offset in [0, 0.5, 0.999999, 1, 1.000001, 59, 60, 119.9999, 120, 121,
                   86399.999, 86400] + [n * 7.123 for n in range(1000)]:
        now = origin + offset
        with patch.object(trigger.time, "time", return_value=now):
            target = trigger.next_wspr_start()
        assert target > now and target - now <= 120
        assert target % 120 == 1
        tested += 1
    # Exercise the actual main/stream/precision-wait loops against a virtual
    # serial port as well, stopping after two complete sends. No real port opens.
    clock = [origin + 50.0]

    class StopSimulation(Exception):
        pass

    class SerialPort:
        def __init__(self):
            self.received = b"GPS STATE: UART=receiving; UTC=valid\n"
            self.sent = []
            self.closed = False

        def __enter__(self):
            return self

        def __exit__(self, *unused):
            self.closed = True

        def fileno(self):
            return 123

        @property
        def in_waiting(self):
            return len(self.received)

        def read(self, count):
            data, self.received = self.received[:count], self.received[count:]
            return data

        def write(self, data):
            self.sent.append((clock[0], data))

        def flush(self):
            if len(self.sent) == 2:
                raise StopSimulation

    port = SerialPort()
    output = io.BytesIO()

    def tick():
        clock[0] += 0.0001
        return clock[0]

    def select_serial(read, write, error, timeout):
        assert read == [123] and not write and not error and 0 <= timeout <= 0.1
        clock[0] += min(timeout, 0.002) if port.received else timeout
        return ([123] if port.received else [], [], [])

    with patch.object(trigger.serial, "Serial", return_value=port, create=True), \
            patch.object(trigger.time, "time", side_effect=tick), \
            patch.object(trigger.select, "select", side_effect=select_serial), \
            patch.object(trigger.sys, "stdout", types.SimpleNamespace(buffer=output)), \
            patch.object(trigger.sys, "stderr", io.StringIO()):
        try:
            trigger.main()
        except StopSimulation:
            pass
    assert port.closed and len(port.sent) == 2
    assert all(data == b"g" for _, data in port.sent)
    assert abs(port.sent[0][0] - (origin + 121)) < 0.001
    assert abs(port.sent[1][0] - (origin + 241)) < 0.001
    assert output.getvalue() == b"GPS STATE: UART=receiving; UTC=valid\n"
    return tested


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, action="append")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--sanitizer", choices=["address", "thread"], default="address")
    parser.add_argument("--output", type=Path, default=ROOT / "build/wspr_simulation")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    compiler_flags = ["cc", "-std=c11", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
                      f"-fsanitize={args.sanitizer},undefined", "-pthread", "-Itests/stubs", "-I."]
    simulator = args.output / "wspr_sim"
    unit_tests = args.output / "wspr_test"
    for source, binary in [("tests/wspr_sim.c", simulator), ("tests/wspr_test.c", unit_tests)]:
        subprocess.run(compiler_flags + [source, "-o", str(binary)], cwd=ROOT, check=True)
    subprocess.run([str(unit_tests)], check=True)
    trigger_cases = check_trigger_script()
    print(f"Python trigger: {trigger_cases} UTC boundary cases and two full send cycles passed", flush=True)
    started = time.monotonic()

    def run(name):
        result = subprocess.run([str(simulator), name], capture_output=True, text=True, timeout=180)
        if result.returncode:
            raise AssertionError(f"{name} exited {result.returncode}: {result.stderr}")
        report = json.loads(result.stdout)
        (args.output / f"{name}.json").write_text(json.dumps(report, indent=2) + "\n")
        return check_report(report)

    results, failures = [], []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        tasks = {pool.submit(run, name): name for name in args.scenario or SCENARIOS}
        for task in as_completed(tasks):
            name = tasks[task]
            try:
                result = task.result()
                results.append(result)
                print(f"PASS {name}: {result['frames']} frames, {result['uart_bytes']} GPS bytes, "
                      f"max symbol error {result['max_symbol_error_us']} us", flush=True)
            except Exception as exc:
                failures.append({"scenario": name, "error": str(exc)})
                print(f"FAIL {name}: {exc}", flush=True)
    results.sort(key=lambda item: SCENARIOS.index(item["scenario"]))
    summary = {"passed": len(results), "failed": failures, "trigger_cases": trigger_cases,
               "trigger_send_cycles": 2, "sanitizer": args.sanitizer,
               "wall_seconds": round(time.monotonic() - started, 3), "scenarios": results,
               "limits": ["Host simulation of production C loops; Pico SDK and hardware are mocked.",
                          "USB backpressure models timeout-zero drops, not TinyUSB internals.",
                          "UART delivery has no receiver latency, oscillator drift or electrical noise unless injected.",
                          "Does not verify RF power, filtering, calibration, reception, PPS timing or real RP2040 scheduling."]}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    lines = ["# WSPR simulation results", "", f"{len(results)} scenarios passed; {len(failures)} failed.",
             f"Python serial trigger: {trigger_cases} UTC boundary checks and two full send cycles passed.",
             f"Sanitizers: {args.sanitizer}, undefined behavior.", "",
             "| Scenario | Frames | GPS bytes | Largest symbol timing error |",
             "| --- | ---: | ---: | ---: |"]
    lines += [f"| {r['scenario']} | {r['frames']} | {r['uart_bytes']} | {r['max_symbol_error_us']} us |" for r in results]
    if failures:
        lines += ["", "Failures:", ""] + [f"- {f['scenario']}: {f['error']}" for f in failures]
    lines += ["", "Limits:", ""] + [f"- {limit}" for limit in summary["limits"]]
    (args.output / "REPORT.md").write_text("\n".join(lines) + "\n")
    print(f"Report: {args.output / 'REPORT.md'}", flush=True)
    return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
