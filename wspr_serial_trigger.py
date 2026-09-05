#!/usr/bin/env python3

import sys
import time
import select
from datetime import datetime, timezone

import serial


PORT = "/dev/cu.usbmodem11401"
BAUD = 115200

# Standard WSPR transmission nominally starts 1 second
# after an even UTC minute.
WSPR_OFFSET_SECONDS = 1.0

# Stop servicing incoming serial data this many seconds
# before the target and concentrate on timing.
FINAL_SPIN_SECONDS = 0.005


def next_wspr_start():
    now = time.time()

    minute_start = int(now // 60) * 60
    minute = datetime.fromtimestamp(
        minute_start, timezone.utc
    ).minute

    if minute % 2 == 0:
        target = minute_start + WSPR_OFFSET_SECONDS

        if target <= now:
            target += 120
    else:
        target = minute_start + 60 + WSPR_OFFSET_SECONDS

    return target


def stream_serial_until(ser, target):
    """
    Stream received serial data to stdout until we're a few
    milliseconds away from the WSPR transmission time.
    """

    fd = ser.fileno()

    while True:
        now = time.time()
        remaining = target - now

        if remaining <= FINAL_SPIN_SECONDS:
            return

        # Don't sleep beyond our final timing window.
        timeout = min(0.100, remaining - FINAL_SPIN_SECONDS)

        readable, _, _ = select.select(
            [fd],
            [],
            [],
            timeout
        )

        if readable:
            waiting = ser.in_waiting

            if waiting:
                data = ser.read(waiting)
            else:
                data = ser.read(1)

            if data:
                # Write exactly what came from the Pico.
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()


def precision_wait(target):
    """
    Busy-wait the last few milliseconds.
    """
    while time.time() < target:
        pass


def main():
    print(
        f"Opening {PORT} at {BAUD} baud...",
        file=sys.stderr
    )

    with serial.Serial(
        PORT,
        BAUD,
        timeout=0,
        write_timeout=1,
    ) as ser:

        print(
            "Serial port open.",
            file=sys.stderr
        )

        print(
            "Streaming Pico output and waiting for WSPR slots.",
            file=sys.stderr
        )

        while True:
            target = next_wspr_start()

            target_dt = datetime.fromtimestamp(
                target,
                timezone.utc
            )

            print(
                f"\nNext 'g': "
                f"{target_dt.strftime('%Y-%m-%d %H:%M:%S.%f UTC')}",
                file=sys.stderr
            )

            # Stream incoming serial traffic for almost the
            # entire waiting period.
            stream_serial_until(ser, target)

            # Concentrate entirely on timing for final ~5 ms.
            precision_wait(target)

            ser.write(b"g")
            ser.flush()

            actual = time.time()
            error_ms = (actual - target) * 1000

            actual_dt = datetime.fromtimestamp(
                actual,
                timezone.utc
            )

            print(
                f"\nSent 'g' at "
                f"{actual_dt.strftime('%H:%M:%S.%f')} UTC "
                f"(timing error {error_ms:+.3f} ms)",
                file=sys.stderr
            )

            # Continue immediately. next_wspr_start()
            # will select the following two-minute slot.


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(
            "\nStopped.",
            file=sys.stderr
        )
