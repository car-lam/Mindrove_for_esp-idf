# basic_test

Test app for the driver. It has two modes, selected with `MODE_STREAM` in `main/main.c`:

- **`0` (default): life-cycle test.** Runs connect → acquire → end `N_CYCLES` times and logs `OK`/`FAIL`
  for every call against its expected return value. It also checks that the package indexes advance by a
  constant step and prints the first packages as CSV. Things to check in the monitor:
  - `mr_connection_end` takes less than ~300 ms;
  - the free heap is the same at the start of cycles 2 and 3 (cycle 1 differs because lwIP and the event
    loop are created once and never freed).
- **`1`: streaming.** Sends every package over the console UART at 921600 baud as a binary frame, for a PC
  script to log:

  | Bytes | Content |
  |---|---|
  | 0–1 | `AA 55` |
  | 2–93 | `mr_data_log_t` (92 B, little-endian): `uint32 idx`, `float emg[2][8]`, `float acc[3]`, `float gyro[3]` |
  | 94–95 | CRC-16/CCITT-FALSE of bytes 2–93, little-endian |

  Boot messages and warnings share the UART with the frames, so the PC side has to skip anything that is
  not a valid frame. Don't keep `idf.py monitor` open at the same time.

## Build and flash

> ⚠️ **Read the warning in the [top-level README](../../README.md#warning-the-armband-has-an-esp32-inside)
> first.** Unplug the armband from USB before flashing.

```sh
cp main/secrets.example.h main/secrets.h   # then fill in the armband's SSID and password
idf.py build
idf.py -p /dev/ttyACM0 flash monitor       # use your dev board's port
```

`sdkconfig.defaults` sets the target to `esp32s3`. The driver is pulled from this repository through
`main/idf_component.yml`, so the example builds from inside the clone without copying anything.
