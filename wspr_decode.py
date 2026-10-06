#!/usr/bin/env python3
"""Decode a Bloonin coarse/fine/altitude/extra-fine-temperature WSPR sequence.

Pass all four locator fields in sequence order, or their raw 15-bit payloads.
Use messages from the same sequence. Coordinates are the extra-fine cell's center; original
position within that cell and rounding/clipping detail cannot be recovered.
"""

import argparse
import json
import re


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

    reversed_longitude, latitude_cell = divmod(coarse_payload, 180)
    longitude_cell = 179 - reversed_longitude
    coarse_grid = (
        chr(ord("A") + longitude_cell // 10)
        + chr(ord("A") + latitude_cell // 10)
        + str(longitude_cell % 10)
        + str(latitude_cell % 10)
    )
    # The displayed fine locator first becomes raw bits, then relative offsets.
    longitude_index, latitude_index = divmod(fine_payload, 180)
    west = -180 + 2 * longitude_cell + longitude_index / 90
    south = -90 + latitude_cell + latitude_index / 180
    extra_index, temperature_code = divmod(packed, 128)
    extra_longitude, extra_latitude = divmod(extra_index, 15)
    latitude_step, longitude_step = 1 / 2700, 1 / 1350
    west += extra_longitude * longitude_step
    south += extra_latitude * latitude_step
    return {
        "latitude": south + latitude_step / 2,
        "longitude": west + longitude_step / 2,
        "altitude_meters": altitude_meters,
        "coarse_grid": coarse_grid,
        "fine_payload": fine_payload,
        "longitude_index": longitude_index,
        "latitude_index": latitude_index,
        "bounds": {
            "south": south,
            "north": south + latitude_step,
            "west": west,
            "east": west + longitude_step,
        },
        "temperature_celsius": temperature_code - 80,
        "temperature_payload": packed,
        "extra_longitude_index": extra_longitude,
        "extra_latitude_index": extra_latitude,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("coarse", help="WSPR_CALLSIGN locator or payload (0..32399)")
    parser.add_argument("fine", help="WSPR_CALLSIGN_FINE locator or payload (0..32399)")
    parser.add_argument("altitude", help="WSPR_CALLSIGN_ALT locator or payload (0..32767)")
    parser.add_argument("extra", help="WSPR_CALLSIGN_TEMP locator or payload (0..28799)")
    parser.add_argument("--json", action="store_true", help="include decoded cell bounds as JSON")
    args = parser.parse_args()
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
