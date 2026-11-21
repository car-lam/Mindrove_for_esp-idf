#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <assert.h>
#include <stdbool.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mindrove_driver.h"
#include "secrets.h" // MR_SSID, MR_PASS: copy secrets.example.h

// Test of the driver's full life cycle: connect -> acquire -> end, several times.
// What to check in the monitor:
//  - every step marked "(expected X)" returns X
//  - mr_connection_end takes < ~300 ms
//  - free heap is the same at the start of cycles 2 and 3 (cycle 1 differs: lwIP/event loop are
//    created once and never freed)

#define N_CYCLES 3
#define N_LOG 100
#define N_PRINT 10 // packages printed per cycle (must be <= N_LOG)

// 1: stream every package over the console UART as binary frames (frame format below; see the example README)
// 0: run the connect/acquire/end cycle test
#define MODE_STREAM 0
#define STREAM_BAUD 921600 // 250 pkt/s x 96 B = 24 KB/s does not fit in 115200 baud (11.5 KB/s)
#define STREAM_POLL_MS 50 // must stay well below N_LOG / 250 pkt/s = 400 ms, or packages get lost

static const char *TAG = "test";

static mr_data_log_t data[N_LOG];

static void report(const char *what, esp_err_t got, esp_err_t expected)
{
    if (got == expected) {
        ESP_LOGI(TAG, "OK   %-35s -> %s", what, esp_err_to_name(got));
    } else {
        ESP_LOGE(TAG, "FAIL %-35s -> %s (expected %s)", what, esp_err_to_name(got), esp_err_to_name(expected));
    }
}

// Checks that package indexes advance by a constant step (no lost packages)
static void check_indexes(void)
{
    uint32_t step = data[1].idx - data[0].idx;
    int jumps = 0;
    for (int k = 1; k < N_LOG; k++) {
        if (data[k].idx - data[k - 1].idx != step) {
            jumps++;
        }
    }
    ESP_LOGI(TAG, "idx: first=%" PRIu32 " last=%" PRIu32 " step=%" PRIu32 " jumps=%d", data[0].idx,
             data[N_LOG - 1].idx, step, jumps);
}

// Prints the first N_PRINT packages in data[]: one line per EMG sample (2 per package), acc and gyro repeated on both
static void print_data(void)
{
    printf("idx,s,emg0,emg1,emg2,emg3,emg4,emg5,emg6,emg7,ax,ay,az,gx,gy,gz\n");
    for (int k = 0; k < N_PRINT; k++) {
        const mr_data_log_t *p = &data[k];
        for (int s = 0; s < MR_N_SAMP_PER_PACK_EMG; s++) {
            printf("%" PRIu32 ",%d", p->idx, s);
            for (int ch = 0; ch < MR_N_EMG_CHANNELS; ch++) {
                printf(",%.2f", p->emg[s][ch]);
            }
            for (int a = 0; a < MR_ACCELEROMETER_CHANNELS; a++) {
                printf(",%.3f", p->acc[a]);
            }
            for (int g = 0; g < MR_GYROSCOPE_CHANNELS; g++) {
                printf(",%.3f", p->gyro[g]);
            }
            printf("\n");
        }
    }
}

// Frame: 0xAA 0x55 | mr_data_log_t (92 B, little-endian) | CRC-16/CCITT-FALSE of the payload (LE)
static_assert(sizeof(mr_data_log_t) == 92, "the frame format assumes a 92-byte package");

static uint16_t crc16_ccitt(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

static void send_frame(const mr_data_log_t *p)
{
    uint8_t frame[2 + sizeof(mr_data_log_t) + 2];
    frame[0] = 0xAA;
    frame[1] = 0x55;
    memcpy(&frame[2], p, sizeof(*p));
    uint16_t crc = crc16_ccitt(&frame[2], sizeof(*p));
    frame[sizeof(frame) - 2] = crc & 0xFF;
    frame[sizeof(frame) - 1] = crc >> 8;
    uart_write_bytes(CONFIG_ESP_CONSOLE_UART_NUM, frame, sizeof(frame));
}

// Polls the driver's ring buffer and sends only the packages not sent yet (by idx).
// Logs share the UART with the frames: routing stdout through the UART driver keeps each frame (one
// uart_write_bytes call) contiguous, and logs are limited to warnings while streaming. The PC side should
// skip anything that is not a valid frame.
static void run_stream(void)
{
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 8192, 0, NULL, 0));
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    // Set here and not in sdkconfig, so boot logs stay at 115200 and nothing can revert it
    ESP_LOGI(TAG, "Console at %d baud from here on", STREAM_BAUD);
    uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);
    ESP_ERROR_CHECK(uart_set_baudrate(CONFIG_ESP_CONSOLE_UART_NUM, STREAM_BAUD));

    while (mr_connect(MR_SSID, MR_PASS, pdMS_TO_TICKS(10000)) != ESP_OK) {
        ESP_LOGW(TAG, "Not connected, retrying");
    }
    ESP_ERROR_CHECK(mr_get_data_start(N_LOG));
    ESP_LOGI(TAG, "Streaming: %d B per frame", (int)(sizeof(mr_data_log_t) + 4));
    esp_log_level_set("*", ESP_LOG_WARN);

    bool any_sent = false;
    uint32_t last_idx = 0;
    while (true) {
        // Until the first fill, poll slowly: each failed call logs "Buffer not full"
        vTaskDelay(pdMS_TO_TICKS(any_sent ? STREAM_POLL_MS : 500));
        if (mr_get_data(data) != ESP_OK) {
            continue; // buffer not full yet, or mutex busy: try again next poll
        }
        for (int k = 0; k < N_LOG; k++) {
            // Signed difference so it keeps working if idx wraps around
            if (!any_sent || (int32_t)(data[k].idx - last_idx) > 0) {
                send_frame(&data[k]);
                last_idx = data[k].idx;
                any_sent = true;
            }
        }
    }
}

static void run_cycle_test(void)
{
    report("end without connect", mr_connection_end(), ESP_OK);

    for (int i = 1; i <= N_CYCLES; i++) {
        ESP_LOGI(TAG, "===== Cycle %d, free heap: %" PRIu32, i, esp_get_free_heap_size());

        report("mr_connect", mr_connect(MR_SSID, MR_PASS, pdMS_TO_TICKS(10000)), ESP_OK);
        report("mr_get_data_start", mr_get_data_start(N_LOG), ESP_OK);
        report("mr_get_data_start (twice)", mr_get_data_start(N_LOG), ESP_ERR_INVALID_STATE);

        vTaskDelay(pdMS_TO_TICKS(3000));

        esp_err_t e = mr_get_data(data);
        report("mr_get_data", e, ESP_OK);
        if (e == ESP_OK) {
            check_indexes();
            print_data();
        }

        TickType_t t0 = xTaskGetTickCount();
        report("mr_connection_end", mr_connection_end(), ESP_OK);
        ESP_LOGI(TAG, "mr_connection_end took %" PRIu32 " ms", (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - t0));

        report("mr_get_data after end", mr_get_data(data), ESP_ERR_INVALID_STATE);
        report("mr_connection_end (twice)", mr_connection_end(), ESP_OK);
    }

    ESP_LOGI(TAG, "===== Done, free heap: %" PRIu32, esp_get_free_heap_size());
}

void app_main(void)
{
#if MODE_STREAM
    run_stream();
#else
    run_cycle_test();
#endif
}
