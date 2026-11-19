"""Run from the repository root: python3 -m unittest discover -s tests -p 'test_wspr_decode.py'."""

import csv
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from wspr_decode import decode_reports, decode_telemetry, locator_to_payload


SCRIPT = Path(__file__).resolve().parents[1] / "wspr_decode.py"


def sequence_rows(start="2026-10-20 20:52:00", variant="X"):
    """The independently checked firmware Munich example, with four receivers."""
    base = datetime.fromisoformat(start).replace(tzinfo=timezone.utc)
    rows = []
    for index, (call, locator) in enumerate((
            ("ZL3XXZ", "JN58"), ("ZL3XXF", "EC31"),
            ("ZL3XXA", "RM93"), ("ZL3XXT", "KR45"))):
        rows.append(dict(id=str(index + 1),
                         time_utc=(base + timedelta(minutes=2 * index)).strftime("%Y-%m-%d %H:%M:%S"),
                         band="14", tx_sign=call.replace("ZL3XX", f"ZL3X{variant}"),
                         tx_loc=locator, rx_sign=f"RX{index}"))
    return rows


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


class CsvDecoderTests(unittest.TestCase):
    def test_full_sequence_merges_receivers_unsorted_rows_and_duplicates(self):
        rows = sequence_rows()
        rows.append(dict(rows[1], id="5", rx_sign="RX_OTHER"))
        rows.append(dict(rows[1]))
        messages, warnings = decode_reports(reversed(rows))
        self.assertEqual(warnings, [])
        self.assertEqual(len(messages), 1)
        message = messages[0]
        expected = decode_telemetry("JN58", "EC31", "RM93", "KR45")
        for key, value in expected.items():
            if key != "bounds":
                self.assertEqual(message[key], value)
        for key, value in expected["bounds"].items():
            self.assertEqual(message[key], value)
        self.assertEqual(message["time_utc"], "2026-10-20 20:52:00")
        self.assertEqual(message["last_report_time_utc"], "2026-10-20 20:58:00")
        self.assertEqual(message["status"], "complete")
        self.assertEqual(message["variant"], "X")
        self.assertEqual(message["position_precision"], "extra_fine")
        self.assertEqual(message["report_count"], 5)
        self.assertEqual(message["receiver_count"], 5)
        self.assertEqual(message["fine_reports"], 2)
        self.assertEqual(message["spot_ids"], "1;2;3;4;5")

    def test_simultaneous_variants_decode_separately(self):
        old = sequence_rows(variant="Y")
        old[0]["tx_loc"] = "FN41"
        # Callsigns are case insensitive in reception CSVs.
        for row in old:
            row["tx_sign"] = row["tx_sign"].lower()
        messages, warnings = decode_reports(reversed(sequence_rows() + old))
        self.assertFalse(warnings)
        self.assertEqual([message["variant"] for message in messages], ["X", "Y"])
        self.assertTrue(all(message["status"] == "complete" for message in messages))
        self.assertEqual([message["coarse_grid"] for message in messages], ["JN58", "FN41"])
        self.assertEqual([message["report_count"] for message in messages], [4, 4])

    def test_variants_cannot_complete_each_others_partial_sequences(self):
        rows = sequence_rows()[:2] + sequence_rows(variant="Y")[2:]
        messages, warnings = decode_reports(rows)
        self.assertFalse(warnings)
        self.assertEqual(len(messages), 2)
        self.assertEqual([message["status"] for message in messages], ["incomplete"] * 2)
        self.assertEqual(messages[0]["missing_callsigns"], "ZL3XXA;ZL3XXT")
        self.assertEqual(messages[1]["missing_callsigns"], "ZL3XYZ;ZL3XYF")
        self.assertIsNone(messages[0]["altitude_meters"])
        self.assertIsNone(messages[1]["latitude"])

    def test_y_conflicts_report_y_callsigns_only(self):
        rows = sequence_rows(variant="Y")
        rows.append(dict(rows[1], id="99", tx_loc="EC32"))
        messages, warnings = decode_reports(rows + sequence_rows())
        self.assertFalse(warnings)
        self.assertEqual(messages[0]["status"], "complete")
        self.assertEqual(messages[1]["variant"], "Y")
        self.assertEqual(messages[1]["conflicting_callsigns"], "ZL3XYF")

    def test_missing_frames_are_not_borrowed_from_next_sequence(self):
        first = sequence_rows()
        second = sequence_rows("2026-10-20 21:00:00")
        second[0]["tx_loc"] = "JN69"
        messages, warnings = decode_reports(first[:2] + second)
        self.assertFalse(warnings)
        partial, complete = messages
        self.assertEqual(partial["status"], "incomplete")
        self.assertEqual(partial["missing_callsigns"], "ZL3XXA;ZL3XXT")
        self.assertIsNone(partial["altitude_meters"])
        self.assertIsNone(partial["temperature_celsius"])
        self.assertEqual(partial["position_precision"], "fine")
        self.assertAlmostEqual(partial["north"] - partial["south"], 1 / 180)
        self.assertEqual(complete["status"], "complete")
        self.assertEqual(complete["coarse_grid"], "JN69")

    def test_orphan_frames_and_missing_fine_do_not_invent_a_position(self):
        rows = sequence_rows()
        messages, _ = decode_reports(rows[1:])
        orphan = messages[0]
        self.assertEqual(orphan["time_utc"], rows[0]["time_utc"])
        self.assertIsNone(orphan["latitude"])
        self.assertIsNone(orphan["west"])
        self.assertEqual(orphan["altitude_meters"], 123)
        self.assertEqual(orphan["temperature_celsius"], 27)
        messages, _ = decode_reports([rows[0], rows[2], rows[3]])
        coarse = messages[0]
        self.assertEqual(coarse["position_precision"], "coarse")
        self.assertEqual(coarse["latitude"], 48.5)
        self.assertEqual(coarse["longitude"], 11)
        self.assertEqual(coarse["temperature_celsius"], 27)

    def test_conflicting_payloads_are_exposed_without_picking_a_receiver(self):
        rows = sequence_rows()
        rows.append(dict(rows[1], id="5", rx_sign="RX_OTHER", tx_loc="EC32"))
        message = decode_reports(rows)[0][0]
        self.assertEqual(message["status"], "conflict")
        self.assertEqual(message["conflicting_callsigns"], "ZL3XXF")
        self.assertEqual(message["fine_locator"], "EC31;EC32")
        self.assertIsNone(message["fine_payload"])
        self.assertEqual(message["position_precision"], "coarse")
        self.assertEqual(message["altitude_meters"], 123)

    def test_zero_payloads_are_present_and_valid(self):
        rows = sequence_rows()
        for row in rows:
            row["tx_loc"] = "RA90"  # Locator payload zero, not missing.
        message = decode_reports(rows)[0][0]
        self.assertEqual(message["status"], "complete")
        self.assertEqual(message["altitude_meters"], 0)
        self.assertEqual(message["temperature_celsius"], -80)
        self.assertEqual(message["fine_payload"], 0)
        self.assertEqual(message["position_precision"], "extra_fine")

    def test_bad_locators_times_and_other_bands(self):
        rows = sequence_rows()
        rows.extend((dict(rows[2], tx_loc="CN89of"),
                     dict(rows[0], time_utc="not a date"),
                     dict(rows[0], time_utc="2026-10-20 20:53:00"),
                     dict(rows[0], tx_sign="UNRELATED"),
                     dict(rows[0], band="7")))
        messages, warnings = decode_reports(rows)
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0]["report_count"], 4)
        self.assertEqual(len(warnings), 3)
        self.assertIn("CSV row 6", warnings[0])
        self.assertIn("even-minute WSPR slot", warnings[2])

    def test_midnight_and_timezone_offsets(self):
        rows = sequence_rows("2026-10-20 23:58:00")
        rows[1]["time_utc"] = "2026-10-21T01:00:00+01:00"
        messages, warnings = decode_reports(rows)
        self.assertFalse(warnings)
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0]["time_utc"], "2026-10-20 23:58:00")
        self.assertEqual(messages[0]["last_report_time_utc"], "2026-10-21 00:04:00")
        self.assertEqual(messages[0]["status"], "complete")

    def test_no_argument_cli_writes_csv_and_json_option_preserves_source(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "wspr_20m.csv"
            rows = sequence_rows()
            rows.extend(sequence_rows(variant="Y"))
            rows.extend(sequence_rows("2026-10-20 21:00:00")[:2])
            conflicting = sequence_rows("2026-10-20 21:08:00")
            conflicting.append(dict(conflicting[1], id="99", tx_loc="EC32"))
            rows.extend(conflicting)
            with source.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0])
                writer.writeheader()
                writer.writerows(rows)
            original = source.read_bytes()
            result = subprocess.run([sys.executable, str(SCRIPT)], cwd=directory,
                                    check=True, capture_output=True, text=True)
            self.assertIn("2 complete", result.stderr)
            self.assertIn("Skipped 2 incomplete/conflicting", result.stderr)
            self.assertIn("Time (UTC)", result.stdout)
            self.assertIn("Variant", result.stdout)
            self.assertIn("48.117222° N", result.stdout)
            self.assertIn("11.516667° E", result.stdout)
            self.assertIn("123 m", result.stdout)
            self.assertIn("27 °C", result.stdout)
            self.assertNotIn("21:00:00", result.stdout)
            self.assertNotIn("21:08:00", result.stdout)
            with (Path(directory) / "wspr_20m_decoded.csv").open(newline="") as stream:
                records = list(csv.DictReader(stream))
            self.assertEqual(len(records), 2)
            self.assertEqual([record["variant"] for record in records], ["X", "Y"])
            self.assertEqual(set(records[0]), {"time_utc", "variant", "latitude", "longitude",
                                               "altitude_meters", "temperature_celsius"})
            self.assertEqual(records[0]["latitude"], "48.117222")
            self.assertEqual(records[0]["longitude"], "11.516667")
            self.assertEqual(records[0]["temperature_celsius"], "27")
            self.assertEqual(records[0]["altitude_meters"], "123")
            self.assertEqual(source.read_bytes(), original)
            custom = Path(directory) / "custom.csv"
            result = subprocess.run([sys.executable, str(SCRIPT), "--input", str(source),
                                     "--output", str(custom), "--json"],
                                    check=True, capture_output=True, text=True)
            decoded = json.loads(result.stdout)
            self.assertEqual(len(decoded), 2)
            self.assertEqual([message["variant"] for message in decoded], ["X", "Y"])
            self.assertEqual(set(decoded[0]), set(records[0]))
            self.assertEqual(decoded[0]["temperature_celsius"], 27)
            self.assertTrue(custom.exists())
            result = subprocess.run([sys.executable, str(SCRIPT), "--input", str(source),
                                     "--output", str(source)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("must differ", result.stderr)
            self.assertEqual(source.read_bytes(), original)

    def test_empty_csv_and_missing_columns(self):
        self.assertEqual(decode_reports([]), ([], []))
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "wspr_20m.csv"
            source.write_text("time_utc,band,tx_sign,tx_loc\n")
            result = subprocess.run([sys.executable, str(SCRIPT)], cwd=directory,
                                    check=True, capture_output=True, text=True)
            self.assertIn("0 complete decodes", result.stderr)
            self.assertEqual(result.stdout.strip(), "No complete telemetry messages yet.")
            source.write_text("wrong,columns\n")
            result = subprocess.run([sys.executable, str(SCRIPT)], cwd=directory,
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("CSV missing required columns", result.stderr)


if __name__ == "__main__":
    unittest.main()
