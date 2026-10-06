"""Run from the repository root: python3 -m unittest discover -s tests -p 'test_wspr_decode.py'."""

import json
from pathlib import Path
import subprocess
import sys
import unittest

from wspr_decode import decode_telemetry, locator_to_payload


class DecoderTests(unittest.TestCase):
    def test_firmware_munich_example(self):
        # Firmware simulation fix: 48°7.038' N, 11°31.000' E, altitude 123 m.
        result = decode_telemetry("JN58", "EC31", "RM93", "KR45")
        self.assertEqual(result["fine_payload"], 24501)
        self.assertEqual((result["longitude_index"], result["latitude_index"]), (136, 21))
        self.assertEqual(result["altitude_meters"], 123)
        self.assertAlmostEqual(result["latitude"], 48.11722222222222)
        self.assertAlmostEqual(result["longitude"], 11.51666666666667)
        bounds = result["bounds"]
        self.assertLessEqual(bounds["south"], 48 + 7.038 / 60)
        self.assertLess(48 + 7.038 / 60, bounds["north"])
        self.assertLessEqual(bounds["west"], 11 + 31 / 60)
        self.assertLess(11 + 31 / 60, bounds["east"])
        self.assertEqual(result, decode_telemetry("15258", "24501", "123", "13675"))
        self.assertEqual(result, decode_telemetry(" jn58 ", "0x5fb5", "0x7b", "0x356b"))

    def test_edges_and_altitude_limits(self):
        southwest = decode_telemetry("AA00", "0", "0", "0")
        self.assertEqual(southwest["bounds"]["south"], -90)
        self.assertEqual(southwest["bounds"]["west"], -180)
        northeast = decode_telemetry("RR99", "32399", "32767", "28799")
        self.assertAlmostEqual(northeast["bounds"]["north"], 90)
        self.assertAlmostEqual(northeast["bounds"]["east"], 180)
        self.assertEqual(northeast["altitude_meters"], 32767)
        self.assertEqual(decode_telemetry("FN41", "0", "32400", "0")["altitude_meters"], 32400)

    def test_relative_offsets_do_not_depend_on_coarse_cell(self):
        first = decode_telemetry("JN58", "16290", "100", "13675")
        second = decode_telemetry("JN69", "16290", "100", "13675")
        self.assertAlmostEqual(second["latitude"] - first["latitude"], 1)
        self.assertAlmostEqual(second["longitude"] - first["longitude"], 2)

    def test_locator_packing(self):
        self.assertEqual(locator_to_payload("RA90"), 0)
        self.assertEqual(locator_to_payload("AR09"), 32399)
        self.assertEqual(locator_to_payload("FN30"), 22810)

    def test_fourth_firmware_message(self):
        # Simulation's first snapshot: extra offsets (7, 1), chip 27 C.
        result = decode_telemetry("JN58", "EC31", "RM93", "KR45")
        self.assertEqual(result["temperature_celsius"], 27)
        self.assertEqual((result["extra_longitude_index"], result["extra_latitude_index"]), (7, 1))
        bounds = result["bounds"]
        for coordinate, lower, upper in ((48 + 7.038 / 60, "south", "north"),
                                         (11 + 31 / 60, "west", "east")):
            self.assertLessEqual(bounds[lower], coordinate)
            self.assertLess(coordinate, bounds[upper])
        self.assertAlmostEqual(bounds["north"] - bounds["south"], 1 / 2700)
        self.assertAlmostEqual(bounds["east"] - bounds["west"], 1 / 1350)

    def test_every_fourth_payload(self):
        # Every transmitted value must survive conversion to a displayed
        # Maidenhead locator and recover independent location/temperature.
        for packed in range(28800):
            reversed_lon, lat = divmod(packed, 180)
            lon = 179 - reversed_lon
            locator = f"{chr(65 + lon // 10)}{chr(65 + lat // 10)}{lon % 10}{lat % 10}"
            result = decode_telemetry("AA00", "0", "0", locator)
            self.assertEqual(result["temperature_payload"], packed)
            self.assertEqual(result["temperature_celsius"], packed % 128 - 80)
            self.assertEqual(result["extra_longitude_index"], packed // 128 // 15)
            self.assertEqual(result["extra_latitude_index"], packed // 128 % 15)
            self.assertAlmostEqual(result["latitude"], -90 + (packed // 128 % 15 + 0.5) / 2700)
            self.assertAlmostEqual(result["longitude"], -180 + (packed // 128 // 15 + 0.5) / 1350)
        northeast = decode_telemetry("RR99", "32399", "32767", "28799")
        self.assertAlmostEqual(northeast["bounds"]["north"], 90)
        self.assertAlmostEqual(northeast["bounds"]["east"], 180)
        for invalid in ("28800", "32767", "-1", "AR09", "bad"):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                decode_telemetry("JN58", "0", "0", invalid)

    def test_invalid_input(self):
        for values in (("ZZ99", "0", "0"), ("JN58aa", "0", "0"),
                       ("JN58", "32400", "0"), ("32400", "0", "0"),
                       ("JN58", "-1", "0"), ("JN58", "0", "32768"),
                       ("JN58", "0", "-1"), ("JN58", "0", "12.5")):
            with self.subTest(values=values), self.assertRaises(ValueError):
                decode_telemetry(*values, "0")

    def test_cli(self):
        script = Path(__file__).resolve().parents[1] / "wspr_decode.py"
        result = subprocess.run(
            [sys.executable, str(script), "JN58", "EC31", "RM93", "KR45", "--json"],
            check=True, text=True, capture_output=True,
        )
        self.assertEqual(json.loads(result.stdout)["altitude_meters"], 123)
        self.assertEqual(json.loads(result.stdout)["temperature_celsius"], 27)
        missing = subprocess.run(
            [sys.executable, str(script), "JN58", "EC31", "RM93"],
            text=True, capture_output=True,
        )
        self.assertEqual(missing.returncode, 2)
        self.assertIn("required: extra", missing.stderr)
        invalid = subprocess.run(
            [sys.executable, str(script), "JN58", "32400", "123", "0"],
            text=True, capture_output=True,
        )
        self.assertEqual(invalid.returncode, 2)
        self.assertIn("fine: payload must be in 0..32399", invalid.stderr)


if __name__ == "__main__":
    unittest.main()
