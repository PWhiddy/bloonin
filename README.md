### Bloonin

The firmware runs the WSPR beacon on CLK0. The default RF tone-zero frequency
is 14.097100 MHz (20 m), configured by `WSPR_BASE_HZ` in `wspr.h`. This is the
actual RF frequency; a receiver uses the 14.095600 MHz USB dial frequency
([WSJT-X WSPR guide](https://wsjt.sourceforge.io/wsjtx-main_en.html#_wspr_mode)).
Set `WSPR_CALLSIGN`, `WSPR_POWER_DBM`, and `WSPR_FALLBACK_GRID` for your station.

```
cmake -S . -B build
cmake --build build
```

read serial with something like:  
```screen /dev/tty.usbmodem11301 115200```

At boot, RF stays off while the C90770 GPS is monitored on UART1 RX GPIO 9 at
9600 baud and USB serial listens for lowercase `g`. Other characters are ignored.
The camera and sweep examples below the beacon call in `bloon.c` are not run.

- A `g` from `wspr_serial_trigger.py` starts the prepared frame immediately.
  Until a GPS fix arrives, the frame uses `WSPR_FALLBACK_GRID`.
- A valid GPS UTC reading and location supply the Maidenhead grid and schedule
  the next start at `hh:mm:01` on an even UTC minute, matching the Python script.
  GPS acquisition alone does not transmit in the middle of a slot.
- After either start, frames repeat every 120 seconds. GPS remains monitored
  during all 162 symbols, and newer fixes update the grid and timing between
  frames. A `g` received during transmission is consumed and ignored.
- If GPS is lost, the beacon retains the last grid and advances its last timing
  anchor using the Pico clock. Fresh GPS readings resynchronize later frames.
  Without either an initial trigger or a GPS time/location fix, RF stays off.

GPS parsing runs on core 1; core 0 handles serial triggers and symbol deadlines.
UTC is anchored to the start of an RMC sentence and includes fractional seconds.
This avoids adding the sentence's full wire time, but it is still NMEA timing:
receiver reporting latency and Pico clock drift remain. There is no PPS input,
so matching the nominal slot does not guarantee exact agreement with host UTC.

Logs report GPS UART/fix/UTC transitions, module text changes, grid updates,
slot synchronization, and RF start/completion/errors. Satellite counts and
checksum errors are summarized only when changed, at most once per ten seconds.
Opening/reconnecting the USB terminal prints a current-state snapshot, even if
boot messages were missed. There is no repeated heartbeat for unchanged state.
`PICO_STDIO_USB_STDOUT_TIMEOUT_US=0`
makes USB output best-effort: a full USB buffer drops remaining log output
instead of waiting for the host. A slow reader can therefore lose or truncate
messages; this setting does not rate-limit logs or eliminate stdio mutex waits.

To use host timing, set `PORT` in `wspr_serial_trigger.py` and run it with Python
and `pyserial` installed. The script streams firmware logs and sends `g` at each
slot. Close other programs using that serial port first.

Host regression checks (simulated time and I2C; no RF hardware required):

```sh
cc -std=c11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined \
  -Itests/stubs -I. tests/wspr_test.c -o /private/tmp/bloonin-wspr-tests
/private/tmp/bloonin-wspr-tests
```
