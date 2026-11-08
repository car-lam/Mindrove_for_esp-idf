# mindrove_driver

Unofficial [ESP-IDF](https://github.com/espressif/esp-idf) component that connects an ESP32 directly to a
**MindRove Armband** over WiFi and decodes its EMG + IMU stream: 8 EMG channels at 500 Hz, accelerometer,
gyroscope and battery level. No PC and no MindRove SDK are needed on the receiving side.

> **Independent project.** Not affiliated with, endorsed by or supported by MindRove Kft. "MindRove" is
> used only to identify the device this driver talks to.

> **Status:** the API is complete and compiles cleanly with `-Wall -Werror` on ESP-IDF v6.1 (ESP32-S3).
> The full connect → acquire → disconnect cycle is still being tested on hardware. Expect changes until
> version 1.0.

## How it works

The armband runs its own WPA2 access point (IP `192.168.4.1`). Once a client joins, the armband sends it
75-byte UDP packets from port 4210 to port 4210 without being asked. That's about 250 packets/s with 2
samples each, i.e. 500 Hz.

The driver:

1. joins the armband's access point as a WiFi station (`mr_connect`);
2. runs two FreeRTOS tasks: one receives the UDP packets, the other decodes them into a ring buffer
   (`mr_get_data_start`);
3. hands you a copy of the ring buffer, oldest package first (`mr_get_data`);
4. stops everything, frees its resources and puts the WiFi back as it was (`mr_connection_end`).

The armband streams to a single client, so disconnect your PC (or any other device) from the armband's
network before connecting the ESP32.

## Installation

Clone or copy this repository into the `components/` folder of your ESP-IDF project:

```sh
cd your_project/components
git clone <this-repo-url> mindrove_driver
```

Then add `mindrove_driver` to the `REQUIRES` (or `PRIV_REQUIRES`) of the component that uses it. `main`
picks up every component automatically if it doesn't list any.

Requirements: ESP-IDF v6.1 (the version it was developed with) and an ESP32 with WiFi. It was developed on
an ESP32-S3. Older 5.x releases may work but have not been tried.

## Usage

```c
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mindrove_driver.h"

#define N_PACKAGES 50 // 50 packages = 100 samples = 200 ms at 500 Hz

static mr_data_log_t window[N_PACKAGES];

void app_main(void)
{
    // SSID and password of the armband's access point
    ESP_ERROR_CHECK(mr_connect("Mindrove_ARB_xxxxxx", "your_password", pdMS_TO_TICKS(10000)));
    ESP_ERROR_CHECK(mr_get_data_start(N_PACKAGES));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (mr_get_data(window) == ESP_OK) {
            // window[0] is the oldest package, window[N_PACKAGES - 1] the newest
            mr_data_log_t *last = &window[N_PACKAGES - 1];
            ESP_LOGI("app", "idx=%lu  ch1=%.1f uV  acc_z=%.2f g", (unsigned long)last->idx,
                     last->emg[1][0], last->acc[2]);
        }
    }

    // When done:
    // mr_connection_end();
}
```

Keep the WiFi credentials out of version control, for example in a `secrets.h` that is listed in
`.gitignore`.

## API

| Function | Description |
|---|---|
| `mr_connect(ssid, pass, ticks)` | Joins the armband's access point and waits up to `ticks` for an IP. Initializes NVS, netif, the default event loop and WiFi if needed. Returns `ESP_OK` right away if already connected. |
| `mr_get_data_start(n)` | Allocates a ring buffer of `n` packages and starts the receive and decode tasks. Returns `ESP_ERR_INVALID_STATE` if it is already running. |
| `mr_get_data(buf)` | Copies the ring buffer into `buf` (must hold `n` packages), oldest first. Returns `ESP_ERR_NOT_FINISHED` until the buffer has filled once. |
| `mr_get_batt_status(&pct)` | Battery level (%) from the last decoded package. |
| `mr_connection_end()` | Stops the tasks, frees the buffer and restores the WiFi (see below). It is safe to call more than once or without a previous `mr_connect`. Also call it after a failed `mr_connect`. |

Each package (`mr_data_log_t`) holds:

| Field | Content | Unit |
|---|---|---|
| `idx` | Sample counter of the first sample in the package | – |
| `emg[2][8]` | 2 consecutive samples × 8 EMG channels | µV |
| `acc[3]` | Accelerometer X, Y, Z | g |
| `gyro[3]` | Gyroscope X, Y, Z | °/s |

### WiFi ownership

The driver uses the ESP32's station interface for as long as it is connected:

- **WiFi not initialized before `mr_connect`:** the driver brings it up and `mr_connection_end` stops and
  deinitializes it.
- **WiFi already initialized by your code:** the driver saves its mode and station config, switches to
  STA (or AP+STA if you were running an access point) and restores both in `mr_connection_end`. It doesn't
  reconnect you to your previous network; call `esp_wifi_connect()` yourself if you need to.

## Packet format

The format was worked out by capturing the armband's own UDP traffic. It was checked against the output of
the official MindRove SDK 5.3.0, and all 4994 samples in the capture matched. The driver contains no SDK
code.

| Bytes | Content | Encoding | Scale |
|---|---|---|---|
| 0–1 | Header | `02 00` | – |
| 2–49 | EMG, 2 samples × 8 channels | int24 big-endian | × 0.045 µV |
| 50 | Battery | uint8 | % |
| 51–54 | Unknown | – | – |
| 55–60 | Accelerometer X, Y, Z | int16 little-endian | ÷ 16384 → g |
| 61–66 | Gyroscope X, Y, Z | int16 little-endian | ÷ 65.5 → °/s |
| 67–74 | Sample counters of sample 1 and sample 2 | uint32 little-endian | – |

## Known limitations

- `mr_get_data` always returns the whole buffer. It doesn't tell you how many packages are new since the
  last call; compare `idx` values if you need that.
- If the receive task stops because of a socket error that it can't recover from, `mr_get_data` keeps
  returning the last data it had and doesn't report an error.
- If your code had the WiFi initialized but stopped before `mr_connect`, it is left started after
  `mr_connection_end`.

## License

[MIT](LICENSE).
