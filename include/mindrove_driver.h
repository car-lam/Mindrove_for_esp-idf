/*
ESP-IDF driver for the MindRove Armband: connects to the armband's WiFi access point and receives its
EMG + IMU UDP stream.

While connected, the ESP32 station interface is used for the armband, so any other station connection is
dropped. mr_connection_end() restores the previous wifi mode and station config.

Independent project, not affiliated with or endorsed by MindRove Kft.
SPDX-License-Identifier: MIT
*/


#ifndef MINDROVE_DRIVER_H
#define MINDROVE_DRIVER_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep

#define MR_N_SAMP_PER_PACK_EMG 2 // EMG samples per data package
#define MR_N_EMG_CHANNELS 8 // number of EMG channels on Mindrove Armband
#define MR_ACCELEROMETER_CHANNELS 3 // number of accelerometer channels on Mindrove Armband
#define MR_GYROSCOPE_CHANNELS 3 // number of gyroscope channels on Mindrove Armband

typedef struct{
    // package's first sample idx (there are two samples per package)
    uint32_t idx;
    // EMG samples' data, 2 per package
    float emg[MR_N_SAMP_PER_PACK_EMG][MR_N_EMG_CHANNELS];
    // Accelerometer data
    float acc[MR_ACCELEROMETER_CHANNELS];
    // Gyroscope data
    float gyro[MR_GYROSCOPE_CHANNELS];
} mr_data_log_t;

// Connects to the armband's access point and waits up to ticks_to_wait for an IP.
// Initializes NVS, netif, the default event loop and wifi if they are not initialized yet.
// If already connected it returns ESP_OK; if it was connected before and dropped, it reconnects.
esp_err_t mr_connect (const char *mr_wifi_ssid, const char *mr_wifi_pass, TickType_t ticks_to_wait);

// Starts the background tasks that receive and decode packets into an internal ring buffer of
// buffer_size packages (each package holds MR_N_SAMP_PER_PACK_EMG EMG samples).
// Requires mr_connect() first. Returns ESP_ERR_INVALID_STATE if acquisition is already running.
esp_err_t mr_get_data_start(size_t buffer_size);

// Copies the whole ring buffer onto buffer, oldest package first (buffer[0] is the oldest).
// buffer must hold at least the buffer_size packages passed to mr_get_data_start().
// Returns ESP_ERR_NOT_FINISHED until the ring buffer has been filled once.
esp_err_t mr_get_data (mr_data_log_t * buffer);

// Gets the battery level (%) reported in the last decoded package
esp_err_t mr_get_batt_status (uint8_t *out_battery_level);

// Terminates connection with mindrove
// Stops data acquisition, frees its resources and leaves the wifi as it was before mr_connect():
//  - if the driver initialized the wifi, it stops and deinitializes it
//  - if the wifi was already in use, it restores the previous mode and station config (but does not reconnect)
// Safe to call more than once, or without mr_connect(). Also call it if mr_connect() failed.
// Returns ESP_ERR_TIMEOUT if the acquisition tasks did not stop; in that case nothing is released and it can be
// called again.
esp_err_t mr_connection_end (void);

#endif
