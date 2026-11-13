#!/usr/bin/env python3
"""Maintain a CSV of 20 m reception reports for the four Bloonin transmitters.

Python 3.9+; standard library only. Data/API documentation: https://wspr.live/
Run without arguments to backfill all available history and poll indefinitely.
"""

import argparse
import csv
from datetime import datetime, timezone
import fcntl
import http.client
import json
import logging
import math
import os
from pathlib import Path
import sqlite3
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request


CALLSIGNS = ("ZL3XYZ", "ZL3XYF", "ZL3XYA", "ZL3XYT")
ENDPOINT = "https://db1.wspr.live/"
EARLIEST = "2008-03-01"
BAND = 14  # WSPR Live uses MHz band identifiers: 14 means 20 metres.
PAGE_SIZE = 10000
REQUEST_GAP = 4.0  # Below the public API's 20 requests/minute limit.
DAY = 86400
FIELDS = (
    "id", "time_utc", "band", "rx_sign", "rx_lat", "rx_lon", "rx_loc",
    "tx_sign", "tx_lat", "tx_lon", "tx_loc", "distance", "azimuth",
    "rx_azimuth", "frequency", "power", "snr", "drift", "version", "code",
)
LOG = logging.getLogger("wspr_collect")


def timestamp(value):
    """Parse a date or ISO timestamp, interpreting an omitted timezone as UTC."""
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        return int(parsed.timestamp())
    except (ValueError, OverflowError) as exc:
        raise argparse.ArgumentTypeError("expected YYYY-MM-DD or an ISO timestamp") from exc


def utc(value):
    return datetime.fromtimestamp(value, timezone.utc).strftime("%Y-%m-%d %H:%M:%S")


def previous_month_start(end):
    """Start of the calendar month containing the last second of [start, end)."""
    date = datetime.fromtimestamp(end - 1, timezone.utc)
    return int(date.replace(day=1, hour=0, minute=0, second=0, microsecond=0).timestamp())


def query_for(start, end, after=None, page_size=PAGE_SIZE):
    columns = ["toString(time, 'UTC') AS time_utc" if col == "time_utc" else col
               for col in FIELDS]
    calls = ", ".join("'" + call + "'" for call in CALLSIGNS)
    query = (
        f"SELECT {', '.join(columns)} FROM wspr.rx "
        f"WHERE band = {BAND} AND tx_sign IN ({calls}) "
        f"AND time >= toDateTime({int(start)}, 'UTC') "
        f"AND time < toDateTime({int(end)}, 'UTC') "
    )
    if after is not None:
        query += f"AND (time, id) > (toDateTime({int(after[0])}, 'UTC'), {int(after[1])}) "
    return query + f"ORDER BY time, id LIMIT {int(page_size)} FORMAT JSON"


class FetchError(Exception):
    """An unsuccessful or incomplete API response; never advance its checkpoint."""


class Client:
    def __init__(self, timeout=60):
        self.timeout = timeout
        self.last_request = -math.inf

    def fetch(self, query):
        time.sleep(max(0, self.last_request + REQUEST_GAP - time.monotonic()))
        self.last_request = time.monotonic()
        url = ENDPOINT + "?" + urllib.parse.urlencode({"query": query})
        request = urllib.request.Request(url, headers={"User-Agent": "bloonin-wspr-collector/1"})
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                result = json.load(response)
        except urllib.error.HTTPError as exc:
            with exc:
                detail = exc.read(500).decode("utf-8", errors="replace")
            raise FetchError(f"HTTP {exc.code}: {detail.strip()}") from exc
        except (OSError, http.client.HTTPException, ValueError) as exc:
            raise FetchError(str(exc)) from exc
        if not isinstance(result, dict) or "exception" in result:
            raise FetchError(f"invalid API response: {str(result)[:500]}")
        rows = result.get("data")
        if not isinstance(rows, list) or result.get("rows") != len(rows):
            raise FetchError("API response has missing/inconsistent rows")
        return rows


class Store:
    """SQLite is authoritative; the CSV is an atomically replaced snapshot."""

    def __init__(self, output, since, now):
        self.output = Path(output).expanduser().resolve()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.database = self.output.with_name(self.output.name + ".sqlite3")
        self.lock = self.output.with_name(self.output.name + ".lock").open("a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.lock.close()
            raise ValueError(f"another collector is already using {self.output}") from None
        self.db = None
        try:
            # Do not silently replace an unrelated CSV or lose it if its sidecar moved.
            if self.output.exists() and not self.database.exists():
                raise ValueError(f"{self.output} exists without its SQLite sidecar; use another --output")
            self.db = sqlite3.connect(self.database)
            self.db.executescript("""
                CREATE TABLE IF NOT EXISTS spots (
                    id TEXT PRIMARY KEY, time_utc TEXT NOT NULL, payload TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS spots_time ON spots(time_utc);
                CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL);
            """)
            stored = self.db.execute("SELECT value FROM metadata WHERE key = 'state'").fetchone()
            if stored:
                self.state = json.loads(stored[0])
                if since is not None and since != self.state["since"]:
                    raise ValueError("--since differs from the saved start date; use another --output")
            else:
                since = timestamp(EARLIEST) if since is None else since
                if since < timestamp(EARLIEST) or since >= now:
                    raise ValueError(f"--since must be on/after {EARLIEST} and before now")
                boundary = max(since, now - 2 * DAY)
                self.state = {"since": since, "live_through": boundary,
                              "history_before": boundary, "rescan_at": 0}
                self.save()
            # Recover a missing/stale CSV after interruption, even if no new spots arrive.
            self.export()
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.db is not None:
            self.db.close()
        self.lock.close()

    def save(self):
        with self.db:
            self.db.execute("INSERT OR REPLACE INTO metadata VALUES ('state', ?)",
                            (json.dumps(self.state),))

    def add(self, rows):
        before = self.db.total_changes
        with self.db:
            self.db.executemany("""
                INSERT INTO spots VALUES (?, ?, ?)
                ON CONFLICT(id) DO UPDATE SET time_utc=excluded.time_utc, payload=excluded.payload
                WHERE spots.payload != excluded.payload
            """, [(str(row["id"]), row["time_utc"], json.dumps(row, sort_keys=True)) for row in rows])
        changed = self.db.total_changes - before
        if changed:
            self.export()
        return changed

    def export(self):
        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", newline="", encoding="utf-8",
                                             dir=self.output.parent, prefix=self.output.name + ".",
                                             suffix=".tmp", delete=False) as stream:
                temp_path = Path(stream.name)
                writer = csv.DictWriter(stream, fieldnames=FIELDS)
                writer.writeheader()
                for (payload,) in self.db.execute("SELECT payload FROM spots ORDER BY time_utc, length(id), id"):
                    writer.writerow(json.loads(payload))
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp_path, self.output)
        finally:
            if temp_path is not None:
                temp_path.unlink(missing_ok=True)


def sync_window(client, store, start, end, page_size=PAGE_SIZE):
    """Page by (time, id), preserving separate receivers for the same transmission."""
    after = None
    changed = 0
    while start < end:
        rows = client.fetch(query_for(start, end, after, page_size))
        # Validate the entire page before storing it, including pagination order.
        previous = after
        clean = []
        try:
            for row in rows:
                row = {key: row[key] for key in FIELDS}
                row["id"] = str(int(row["id"]))
                key = (timestamp(row["time_utc"]), int(row["id"]))
                if (row["band"] != BAND or row["tx_sign"] not in CALLSIGNS
                        or not start <= key[0] < end or key[1] < 0
                        or (after is not None and key <= after)
                        or (previous is not None and key < previous)):
                    raise ValueError("out-of-range or out-of-order spot")
                row["time_utc"] = utc(key[0])
                previous = key
                clean.append(row)
        except (KeyError, TypeError, ValueError, argparse.ArgumentTypeError) as exc:
            raise FetchError(f"invalid spot response: {exc}") from exc
        changed += store.add(clean)
        if len(rows) < page_size:
            break
        after = previous
    LOG.info("%s to %s UTC: %d new/updated reports", utc(start), utc(end), changed)


def sync_live(client, store, target, overlap):
    start = max(store.state["since"], store.state["live_through"] - overlap)
    while start < target:
        end = min(start + 31 * DAY, target)
        sync_window(client, store, start, end)
        store.state["live_through"] = max(store.state["live_through"], end)
        store.save()
        start = end


def run(args):
    target = int(time.time())
    store = Store(args.output, args.since, target)
    try:
        client = Client(args.timeout)
        LOG.info("Tracking %s on 20 m; CSV: %s", ", ".join(CALLSIGNS), store.output)
        LOG.info("History starts %s UTC; saved progress resumes automatically", utc(store.state["since"]))
        if args.rescan:
            store.state["history_before"] = store.state["live_through"]
            store.save()
        next_poll = 0
        failures = 0
        while True:
            try:
                if time.monotonic() >= next_poll:
                    sync_live(client, store, target if args.once else int(time.time()),
                              int(args.overlap_hours * 3600))
                    next_poll = time.monotonic() + args.interval
                if store.state["history_before"] > store.state["since"]:
                    end = store.state["history_before"]
                    start = max(store.state["since"], previous_month_start(end))
                    sync_window(client, store, start, end)
                    store.state["history_before"] = start
                    if start == store.state["since"]:
                        store.state["rescan_at"] = int(time.time() + args.rescan_days * DAY)
                        LOG.info("Historical scan complete")
                    store.save()
                elif args.once:
                    return 0
                elif time.time() >= store.state["rescan_at"]:
                    # Revisit older history too: arbitrarily late uploads/reimports can
                    # fall outside the recent polling overlap.
                    store.state["history_before"] = store.state["live_through"]
                    store.save()
                    LOG.info("Starting scheduled historical rescan")
                else:
                    time.sleep(min(60, max(0, next_poll - time.monotonic()),
                                   max(0, store.state["rescan_at"] - time.time())))
                failures = 0
            except FetchError as exc:
                failures += 1
                if args.once and failures >= 5:
                    LOG.error("API failed after five attempts: %s", exc)
                    return 1
                delay = min(300, 5 * 2 ** min(failures - 1, 6))
                LOG.warning("API error: %s; retrying in %ds (progress saved)", exc, delay)
                time.sleep(delay)
    finally:
        store.close()


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("wspr_20m.csv"), help="CSV path (default: %(default)s)")
    parser.add_argument("--since", type=timestamp,
                        help=f"UTC history start, e.g. 2026-11-01 (default: saved value or {EARLIEST})")
    parser.add_argument("--interval", type=positive, default=120, help="poll interval in seconds (default: %(default)s)")
    parser.add_argument("--overlap-hours", type=positive, default=48, help="recheck this many recent hours (default: %(default)s)")
    parser.add_argument("--rescan-days", type=positive, default=7, help="days between full history rescans (default: %(default)s)")
    parser.add_argument("--timeout", type=positive, default=60, help="HTTP timeout in seconds (default: %(default)s)")
    parser.add_argument("--once", action="store_true", help="catch up and finish pending historical backfill, then exit")
    parser.add_argument("--rescan", action="store_true", help="restart the historical scan without discarding existing reports")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        return run(args)
    except KeyboardInterrupt:
        LOG.info("Stopped; rerun the same command to resume")
        return 130
    except (OSError, sqlite3.Error, ValueError) as exc:
        LOG.error("%s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
