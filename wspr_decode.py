#!/usr/bin/env python3
"""Decode a Bloonin coarse/fine/altitude/extra-fine-temperature WSPR sequence.

With no positional arguments, print complete decoded messages as a readable
table and save their final values to wspr_20m_decoded.csv, reading wspr_20m.csv.
Alternatively, pass all four locator fields in sequence order, or their raw
15-bit payloads. Coordinates are the decoded cell's center; original position
within that cell and rounding/clipping detail cannot be recovered.
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import sys
import tempfile


# Same order as wspr_beacon_message(); each frame uses the same frozen fix.
FRAMES = (
    ("coarse", "ZL3XYZ", 32399),
    ("fine", "ZL3XYF", 32399),
    ("altitude", "ZL3XYA", 32767),
    ("extra", "ZL3XYT", 28799),
)
SLOT_SECONDS = 120
CSV_FIELDS = (
    "time_utc", "latitude", "longitude", "altitude_meters", "temperature_celsius",
)


def locator_to_payload(locator: str) -> int:
    """Recover the raw locator bits using the packing rule in wspr.h."""
    locator = locator.strip().upper()
    if not re.fullmatch(r"[A-R]{2}[0-9]{2}", locator):
        raise ValueError(f"invalid four-character Maidenhead locator: {locator!r}")
    longitude = 10 * (ord(locator[0]) - ord("A")) + int(locator[2])
    latitude = 10 * (ord(locator[1]) - ord("A")) + int(locator[3])
    return (179 - longitude) * 180 + latitude


def parse_payload(value: str, field: str, maximum: int) -> int:
    """Accept a displayed locator, decimal integer, or 0x-prefixed integer."""
    value = value.strip()
    if re.fullmatch(r"[A-Ra-r]{2}[0-9]{2}", value):
        payload = locator_to_payload(value)
    else:
        try:
            payload = int(value, 16 if value.lower().startswith("0x") else 10)
        except ValueError as error:
            raise ValueError(
                f"{field}: expected a four-character locator or an integer payload, got {value!r}"
            ) from error
    if not 0 <= payload <= maximum:
        raise ValueError(f"{field}: payload must be in 0..{maximum}, got {payload}")
    return payload


def decode_telemetry(coarse: str, fine: str, altitude: str, extra: str) -> dict:
    """Decode locator strings or numeric strings from one GPS snapshot.

    Latitude/longitude are decimal degrees (north/east positive). Altitude is
    the transmitted GGA altitude in meters, rounded/clamped by the firmware.
    The fourth field adds 15x15 refinement and Pico die temperature in °C.
    Bounds are south/west inclusive, north/east exclusive.
    """
    coarse_payload = parse_payload(coarse, "coarse", 32399)
    fine_payload = parse_payload(fine, "fine", 32399)
    altitude_meters = parse_payload(altitude, "altitude", 32767)
    packed = parse_payload(extra, "extra-fine/temperature", 28799)
    return decode_payloads(coarse_payload, fine_payload, altitude_meters, packed)


def decode_payloads(coarse_payload, fine_payload, altitude_meters, packed):
    """Decode validated payloads, allowing None for absent or ambiguous frames."""
    result = {
        "latitude": None,
        "longitude": None,
        "altitude_meters": altitude_meters,
        "coarse_grid": None,
        "fine_payload": fine_payload,
        "longitude_index": None,
        "latitude_index": None,
        "bounds": dict.fromkeys(("south", "north", "west", "east")),
        "temperature_celsius": None,
        "temperature_payload": packed,
        "extra_longitude_index": None,
        "extra_latitude_index": None,
    }
    if fine_payload is not None:
        longitude_index, latitude_index = divmod(fine_payload, 180)
        result.update(longitude_index=longitude_index, latitude_index=latitude_index)
    if packed is not None:
        extra_index, temperature_code = divmod(packed, 128)
        extra_longitude, extra_latitude = divmod(extra_index, 15)
        result.update(temperature_celsius=temperature_code - 80,
                      extra_longitude_index=extra_longitude,
                      extra_latitude_index=extra_latitude)
    if coarse_payload is not None:
        reversed_longitude, latitude_cell = divmod(coarse_payload, 180)
        longitude_cell = 179 - reversed_longitude
        result["coarse_grid"] = (
            chr(ord("A") + longitude_cell // 10)
            + chr(ord("A") + latitude_cell // 10)
            + str(longitude_cell % 10) + str(latitude_cell % 10)
        )
        west, south = -180 + 2 * longitude_cell, -90 + latitude_cell
        longitude_step, latitude_step = 2, 1
        if fine_payload is not None:
            west += longitude_index / 90
            south += latitude_index / 180
            longitude_step, latitude_step = 1 / 90, 1 / 180
            if packed is not None:
                longitude_step, latitude_step = 1 / 1350, 1 / 2700
                west += extra_longitude * longitude_step
                south += extra_latitude * latitude_step
        result.update(latitude=south + latitude_step / 2, longitude=west + longitude_step / 2,
                      bounds={"south": south, "north": south + latitude_step,
                              "west": west, "east": west + longitude_step})
    return result


def utc_string(timestamp):
    return datetime.fromtimestamp(timestamp, timezone.utc).strftime("%Y-%m-%d %H:%M:%S")


def decode_reports(rows):
    """Return chronological sequence records and diagnostics for invalid rows.

    A frame's callsign determines its offset from the coarse frame. Never carry
    a field across sequences or infer a global eight-minute UTC alignment.
    Conflicting payloads remain ambiguous rather than choosing a receiver.
    """
    by_call = {call: (index, name, maximum)
               for index, (name, call, maximum) in enumerate(FRAMES)}
    groups = {}
    warnings = []
    for row_number, row in enumerate(rows, 2):
        try:
            call = row["tx_sign"].strip().upper()
            if call not in by_call or int(row["band"]) != 14:
                continue
            index, name, maximum = by_call[call]
            date = datetime.fromisoformat(row["time_utc"].strip().replace("Z", "+00:00"))
            if date.tzinfo is None:
                date = date.replace(tzinfo=timezone.utc)
            when = date.timestamp()
            if when % SLOT_SECONDS:
                raise ValueError("time_utc must identify an even-minute WSPR slot")
            when = int(when)
            locator = row["tx_loc"].strip().upper()
            payload = parse_payload(locator, name, maximum)
            receiver = (row.get("rx_sign") or "").strip().upper()
            spot_id = (row.get("id") or "").strip()
        except (KeyError, AttributeError, TypeError, ValueError, OverflowError) as error:
            warnings.append(f"CSV row {row_number}: {error}; skipped")
            continue
        start = when - index * SLOT_SECONDS
        frames = groups.setdefault(start, {})
        variants = frames.setdefault(name, {})
        report = variants.setdefault(payload, {"locators": set(), "reports": set(),
                                                "receivers": set(), "ids": set()})
        report["locators"].add(locator)
        # Duplicate CSV rows must not inflate report counts.
        report["reports"].add(("id", spot_id) if spot_id else ("receiver", receiver))
        if receiver:
            report["receivers"].add(receiver)
        if spot_id:
            report["ids"].add(spot_id)

    messages = []
    for start, frames in sorted(groups.items()):
        payloads = [next(iter(frames[name])) if len(frames.get(name, {})) == 1 else None
                    for name, _, _ in FRAMES]
        decoded = decode_payloads(*payloads)
        bounds = decoded.pop("bounds")
        missing = [call for name, call, _ in FRAMES if name not in frames]
        conflicting = [call for name, call, _ in FRAMES if len(frames.get(name, {})) > 1]
        precision = ("" if payloads[0] is None else "coarse" if payloads[1] is None
                     else "fine" if payloads[3] is None else "extra_fine")
        last_index = max(index for index, (name, _, _) in enumerate(FRAMES) if name in frames)
        message = dict(time_utc=utc_string(start),
                       last_report_time_utc=utc_string(start + last_index * SLOT_SECONDS),
                       status="conflict" if conflicting else "incomplete" if missing else "complete",
                       missing_callsigns=";".join(missing), conflicting_callsigns=";".join(conflicting),
                       position_precision=precision, **decoded, **bounds)
        receivers, ids, reports = set(), set(), set()
        for name, _, _ in FRAMES:
            locators, frame_reports = set(), set()
            for variant in frames.get(name, {}).values():
                locators.update(variant["locators"])
                frame_reports.update(variant["reports"])
                receivers.update(variant["receivers"])
                ids.update(variant["ids"])
            reports.update((name, report) for report in frame_reports)
            message[name + "_locator"] = ";".join(sorted(locators))
            message[name + "_reports"] = len(frame_reports)
        message.update(report_count=len(reports), receiver_count=len(receivers),
                       receivers=";".join(sorted(receivers)), spot_ids=";".join(sorted(ids)))
        messages.append(message)
    return messages, warnings


def read_reports(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = {"time_utc", "band", "tx_sign", "tx_loc"} - set(reader.fieldnames or [])
        if missing:
            raise ValueError("CSV missing required columns: " + ", ".join(sorted(missing)))
        return decode_reports(reader)


def successful_messages(sequences):
    """Keep only complete, unambiguous fixes and their final physical units."""
    messages = []
    for sequence in sequences:
        if sequence["status"] != "complete":
            continue
        message = {field: sequence[field] for field in CSV_FIELDS}
        message["latitude"] = round(message["latitude"], 6)
        message["longitude"] = round(message["longitude"], 6)
        messages.append(message)
    return messages


def print_messages(messages):
    if not messages:
        print("No complete telemetry messages yet.")
        return
    headers = ("Time (UTC)", "Latitude", "Longitude", "Altitude", "Chip temperature")
    rows = [headers]
    for message in messages:
        latitude, longitude = message["latitude"], message["longitude"]
        rows.append((message["time_utc"],
                     f"{abs(latitude):.6f}° {'N' if latitude >= 0 else 'S'}",
                     f"{abs(longitude):.6f}° {'E' if longitude >= 0 else 'W'}",
                     f"{message['altitude_meters']} m",
                     f"{message['temperature_celsius']} °C"))
    widths = [max(len(row[index]) for row in rows) for index in range(len(headers))]
    for row in rows:
        print("  ".join(value.ljust(width) for value, width in zip(row, widths)).rstrip())


def write_messages(path, messages):
    """Replace the parsed CSV atomically, leaving the collector's CSV untouched."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="",
                                         dir=path.parent, prefix=path.name + ".",
                                         suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for message in messages:
                writer.writerow(dict(message, latitude=f"{message['latitude']:.6f}",
                                     longitude=f"{message['longitude']:.6f}"))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("coarse", nargs="?", help="WSPR_CALLSIGN locator or payload (0..32399)")
    parser.add_argument("fine", nargs="?", help="WSPR_CALLSIGN_FINE locator or payload (0..32399)")
    parser.add_argument("altitude", nargs="?", help="WSPR_CALLSIGN_ALT locator or payload (0..32767)")
    parser.add_argument("extra", nargs="?", help="WSPR_CALLSIGN_TEMP locator or payload (0..28799)")
    parser.add_argument("--input", type=Path, help="source CSV (default: wspr_20m.csv)")
    parser.add_argument("--output", type=Path, help="parsed CSV (default: <input stem>_decoded.csv)")
    parser.add_argument("--json", action="store_true", help="print decoded data as JSON; in CSV mode, also write the parsed CSV")
    args = parser.parse_args()
    values = [args.coarse, args.fine, args.altitude, args.extra]
    if not any(value is not None for value in values):
        source = (args.input or Path("wspr_20m.csv")).expanduser().resolve()
        output = (args.output or source.with_name(source.stem + "_decoded.csv")).expanduser().resolve()
        if source == output or (output.exists() and source.exists() and output.samefile(source)):
            parser.error("--output must differ from the input CSV")
        try:
            sequences, warnings = read_reports(source)
            messages = successful_messages(sequences)
            write_messages(output, messages)
        except (OSError, ValueError, csv.Error) as error:
            parser.error(str(error))
        print(f"Saved {len(messages)} complete decodes to {output}. "
              f"Skipped {len(sequences) - len(messages)} incomplete/conflicting sequences "
              f"and {len(warnings)} invalid reports.", file=sys.stderr)
        if args.json:
            print(json.dumps(messages, indent=2))
        else:
            print_messages(messages)
        return
    missing = [name for (name, _, _), value in zip(FRAMES, values) if value is None]
    if missing:
        parser.error("the following arguments are required: " + ", ".join(missing))
    if args.input is not None or args.output is not None:
        parser.error("--input/--output apply only when no locator arguments are supplied")
    try:
        position = decode_telemetry(args.coarse, args.fine, args.altitude, args.extra)
    except ValueError as error:
        parser.error(str(error))
    if args.json:
        print(json.dumps(position, indent=2))
    else:
        print(f"Latitude:  {position['latitude']:.6f}")
        print(f"Longitude: {position['longitude']:.6f}")
        print(f"Altitude:  {position['altitude_meters']} m")
        print(f"Temperature: {position['temperature_celsius']} °C (Pico chip)")
        print("Coordinates are the extra-fine cell center (1/2700° latitude × 1/1350° longitude).")


if __name__ == "__main__":
    main()
