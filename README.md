### Bloonin

The firmware runs the WSPR beacon on CLK0. The default RF tone-zero frequency
is 14.097100 MHz (20 m), configured by `WSPR_BASE_HZ` in `wspr.h`. This is the
actual RF frequency; a receiver uses the 14.095600 MHz USB dial frequency
([WSJT-X WSPR guide](https://wsjt.sourceforge.io/wsjtx-main_en.html#_wspr_mode)).

```
cmake -S . -B build
cmake --build build
```

read serial with something like:  
```screen /dev/tty.usbmodem11301 115200```

At boot, RF stays off while the C90770 GPS is monitored on UART1 RX GPIO 9 at
9600 baud and USB serial listens for lowercase `g`. Other characters are ignored.
The camera and sweep examples below the beacon call in `bloon.c` are not run.

- A `g` from `wspr_serial_trigger.py` starts a standard frame immediately,
  using `WSPR_FALLBACK_GRID`. Standard frames repeat every 120 seconds until GPS
  takes over. A `g` received during transmission is consumed and ignored.
- A valid GPS UTC reading and complete location/altitude fix schedule
  the next start at `hh:mm:01` on an even UTC minute, matching the Python script.
  GPS acquisition alone does not transmit in the middle of a slot.
- GPS operation cycles through standard grid, fine grid, and altitude frames,
  one every 120 seconds. At the standard frame's start, coordinates and altitude
  are frozen from the latest complete GGA fix for all three messages. GPS
  remains monitored during transmission. Once GPS takes over, serial triggers
  are ignored, including during subsequent GPS outages.
- If GPS is lost, the current three-message sequence finishes using its frozen
  fix and the Pico clock. Subsequent slots are skipped until fresh GPS returns;
  the next sequence starts with a standard frame on an even-minute slot. A new
  sequence requires both UTC and a complete fix less than five seconds old.
  Without an initial serial trigger or complete GPS fix plus UTC, RF stays off.

The callsign identifies the payload stored in the 15-bit locator field:

| Callsign setting | Locator-field payload |
| --- | --- |
| `WSPR_CALLSIGN` | Standard four-character Maidenhead grid |
| `WSPR_CALLSIGN_FINE` | `longitude_index * 180 + latitude_index` (0–32399) |
| `WSPR_CALLSIGN_ALT` | GGA altitude in meters, rounded to nearest meter and clamped to 0–32767 |

Fine indices run from 0 to 179 eastward and northward from the southwest corner
of the **frozen coarse cell**, dividing its 2° longitude by 1° latitude extent
into 180 × 180 bins. Decode with `longitude_index = payload / 180` (integer
division) and `latitude_index = payload % 180`. Power and channel coding stay
unchanged; telemetry decoding must interpret the raw locator bits according to
the callsign, rather than treating fine/altitude frames as geographic locators.

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
python3 tests/run_simulation.py
```
