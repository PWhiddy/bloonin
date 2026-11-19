"""Offline regression tests for collection, persistence, and API failures."""

import csv
from contextlib import closing
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import urllib.error

import wspr_collect as collector


START = collector.timestamp("2026-11-01")


def spot(number, second=0, receiver="RX1", call="ZL3XXZ"):
    row = dict.fromkeys(collector.FIELDS, 0)
    row.update(id=str(number), time_utc=collector.utc(START + second), band=14,
               tx_sign=call, tx_loc="EC31", rx_sign=receiver, rx_loc="JN58aa",
               frequency=14097100, snr=-24, power=10, version="test", code=1)
    return row


class FakeClient:
    def __init__(self, *responses):
        self.responses = iter(responses)
        self.queries = []

    def fetch(self, query):
        self.queries.append(query)
        response = next(self.responses)
        if isinstance(response, Exception):
            raise response
        return response


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.output = Path(self.directory.name) / "spots.csv"
        self.store = collector.Store(self.output, START, START + 3600)
        self.addCleanup(lambda: self.store.close())

    def rows(self):
        with self.output.open(newline="") as stream:
            return list(csv.DictReader(stream))

    def test_pagination_keeps_all_receivers_and_calls_at_same_timestamp(self):
        rows = [spot(i, receiver=f"RX{i}", call=call)
                for i, call in enumerate(collector.CALLSIGNS, 1)]
        client = FakeClient(*(rows[i:i + 2] for i in range(0, len(rows), 2)), [])
        collector.sync_window(client, self.store, START, START + 120, page_size=2)
        self.assertEqual([row["id"] for row in self.rows()], [str(i) for i in range(1, 9)])
        self.assertEqual(len(client.queries), 5)
        self.assertIn(f"(time, id) > (toDateTime({START}, 'UTC'), 2)", client.queries[1])
        self.assertIn("band = 14", client.queries[0])
        self.assertIn("tx_sign IN ('ZL3XXZ', 'ZL3XXF', 'ZL3XXA', 'ZL3XXT', "
                      "'ZL3XYZ', 'ZL3XYF', 'ZL3XYA', 'ZL3XYT')", client.queries[0])
        self.assertEqual({row["tx_sign"] for row in self.rows()}, set(collector.CALLSIGNS))
        self.assertIn("LIMIT 2 FORMAT JSON", client.queries[0])

    def test_restart_deduplication_corrections_and_csv_recovery(self):
        self.store.add([spot(10), spot(2), spot(10)])
        self.assertEqual([row["id"] for row in self.rows()], ["2", "10"])
        self.store.state["live_through"] = START + 600
        self.store.save()
        self.store.close()
        self.output.write_text("interrupted/stale export")
        self.store = collector.Store(self.output, None, START + 7200)
        self.assertEqual(len(self.rows()), 2)
        self.assertEqual(self.store.state["live_through"], START + 600)
        self.assertEqual(self.store.add([spot(10)]), 0)
        updated = spot(10)
        updated["snr"] = -10
        self.assertEqual(self.store.add([updated]), 1)
        self.assertEqual(self.rows()[1]["snr"], "-10")
        self.assertEqual(self.rows()[1]["tx_loc"], "EC31")

    def test_failed_page_keeps_checkpoint_and_replays_safely(self):
        client = FakeClient([spot(1), spot(2)], collector.FetchError("connection lost"))
        with self.assertRaises(collector.FetchError):
            collector.sync_window(client, self.store, START, START + 120, page_size=2)
        self.assertEqual(self.store.state["live_through"], START)
        self.assertEqual(len(self.rows()), 2)
        collector.sync_window(FakeClient([spot(1), spot(2)], [spot(3)]),
                              self.store, START, START + 120, page_size=2)
        self.assertEqual(len(self.rows()), 3)

    def test_legacy_checkpoint_rescans_both_families_once_without_losing_reports(self):
        self.store.add([spot(1, call="ZL3XYZ")])
        for previous_calls in (None, ["ZL3XXZ", "ZL3XXF", "ZL3XXA", "ZL3XXT"]):
            with self.subTest(previous_calls=previous_calls):
                self.store.state.update(live_through=START + 600, history_before=START,
                                        rescan_at=START + collector.DAY)
                if previous_calls is None:
                    self.store.state.pop("callsigns", None)
                else:
                    self.store.state["callsigns"] = previous_calls
                self.store.save()
                self.store.close()
                self.store = collector.Store(self.output, None, START + 7200)
                self.assertEqual(self.store.state["live_through"], START + 600)
                self.assertEqual(self.store.state["history_before"], START + 600)
                self.assertEqual(set(self.store.state["callsigns"]), set(collector.CALLSIGNS))
                self.assertEqual(self.rows()[0]["tx_sign"], "ZL3XYZ")
                # Completing that backfill must not restart it on another launch.
                self.store.state["history_before"] = START
                self.store.save()
                self.store.close()
                self.store = collector.Store(self.output, None, START + 7300)
                self.assertEqual(self.store.state["history_before"], START)

    def test_overlap_finds_delayed_report_and_advances_through_empty_intervals(self):
        collector.sync_live(FakeClient([spot(1)]), self.store, START + 120, 3600)
        delayed = FakeClient([spot(1), spot(2, receiver="LATE")])
        collector.sync_live(delayed, self.store, START + 240, 3600)
        self.assertIn(f"time >= toDateTime({START}, 'UTC')", delayed.queries[0])
        self.assertEqual(len(self.rows()), 2)
        collector.sync_live(FakeClient([]), self.store, START + 360, 3600)
        self.assertEqual(self.store.state["live_through"], START + 360)

    def test_offline_catchup_uses_bounded_windows(self):
        client = FakeClient([], [], [])
        collector.sync_live(client, self.store, START + 70 * collector.DAY, 3600)
        self.assertEqual(len(client.queries), 3)
        self.assertEqual(self.store.state["live_through"], START + 70 * collector.DAY)

    def test_bad_page_is_not_stored(self):
        for update in ({"tx_sign": "OTHER"}, {"band": 7},
                       {"time_utc": collector.utc(START + 120)}, {"id": "bad"}):
            with self.subTest(update=update):
                row = spot(2)
                row.update(update)
                with self.assertRaises(collector.FetchError):
                    collector.sync_window(FakeClient([spot(1), row]), self.store, START, START + 120)
                self.assertEqual(self.rows(), [])

    def test_lock_rejects_second_writer(self):
        with self.assertRaisesRegex(ValueError, "another collector"):
            collector.Store(self.output, None, START + 3600)

    def test_atomic_export_failure_keeps_previous_csv(self):
        self.store.add([spot(1)])
        with patch.object(collector.os, "replace", side_effect=OSError("disk error")):
            with self.assertRaises(OSError):
                self.store.add([spot(2)])
        self.assertEqual(len(self.rows()), 1)
        self.store.close()
        self.store = collector.Store(self.output, None, START + 7200)
        self.assertEqual(len(self.rows()), 2)


class ApiAndCliTests(unittest.TestCase):
    def test_network_errors_and_partial_json_are_failures(self):
        client = collector.Client()
        for error in (urllib.error.URLError("offline"),
                      urllib.error.HTTPError(collector.ENDPOINT, 429, "slow down", {}, io.BytesIO(b"rate limit"))):
            with patch.object(collector.urllib.request, "urlopen", side_effect=error), \
                    patch.object(collector.time, "sleep"):
                with self.assertRaises(collector.FetchError):
                    client.fetch("SELECT 1")
        for body in (b'{"data": [', b'{"data": [], "rows": 1}', b'{"exception": "error"}'):
            with patch.object(collector.urllib.request, "urlopen", return_value=io.BytesIO(body)), \
                    patch.object(collector.time, "sleep"):
                with self.assertRaises(collector.FetchError):
                    client.fetch("SELECT 1")

    def test_public_get_query_and_utc_dates(self):
        with patch.object(collector.urllib.request, "urlopen",
                          return_value=io.BytesIO(json.dumps({"data": [], "rows": 0}).encode())) as request:
            collector.Client().fetch("SELECT 1 FORMAT JSON")
        self.assertEqual(request.call_args.args[0].get_method(), "GET")
        self.assertIn("?query=SELECT+1+FORMAT+JSON", request.call_args.args[0].full_url)
        self.assertEqual(collector.timestamp("2026-11-01T01:00:00+01:00"), START)
        self.assertEqual(collector.timestamp("2026-11-01T00:00:00Z"), START)

    def test_backfill_month_boundaries_and_resumable_once(self):
        since = collector.timestamp("2026-09-15")
        now = START + 3600
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "spots.csv"
            # Recent query, October, then the selected part of September.
            client = FakeClient([], [], [])
            with patch.object(collector.time, "time", return_value=now), \
                    patch.object(collector, "Client", return_value=client):
                self.assertEqual(collector.main(["--output", str(output), "--since", "2026-09-15", "--once"]), 0)
            self.assertEqual(len(client.queries), 3)
            with closing(collector.sqlite3.connect(str(output) + ".sqlite3")) as db:
                state = json.loads(db.execute("SELECT value FROM metadata WHERE key='state'").fetchone()[0])
            self.assertEqual(state["history_before"], since)
            self.assertEqual(state["live_through"], now)
            self.assertGreater(state["rescan_at"], now)
            self.assertEqual(collector.previous_month_start(START), collector.timestamp("2026-10-01"))
            self.assertEqual(collector.previous_month_start(START + 1), START)
            # Restart checks recent reports without scanning old months again.
            resumed = FakeClient([])
            with patch.object(collector.time, "time", return_value=now + 120), \
                    patch.object(collector, "Client", return_value=resumed):
                self.assertEqual(collector.main(["--output", str(output), "--once"]), 0)
            self.assertEqual(len(resumed.queries), 1)

    def test_existing_csv_without_sidecar_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "existing.csv"
            output.write_text("keep me")
            with self.assertRaisesRegex(ValueError, "without its SQLite sidecar"):
                collector.Store(output, START, START + 3600)
            self.assertEqual(output.read_text(), "keep me")


if __name__ == "__main__":
    unittest.main()
