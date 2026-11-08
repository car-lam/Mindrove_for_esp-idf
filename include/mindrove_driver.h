/*
This component establishes connection with and samples data from the Mindrove Armband.
The execution of this component will stop any other wifi connection
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

// Start connection. This function will verify whether the wifi connection has been already established, and, if not, it will.
esp_err_t mr_connect (const char *mr_wifi_ssid, const char *mr_wifi_pass, TickType_t ticks_to_wait);

// Starts a task that gets data continuously
// This function calls mr_get_data once the buffer is full so the user only works with complete buffers
esp_err_t mr_get_data_start(size_t buffer_size);

// Writes the stored data onto a buffer
// The first sample (idx 0) is the oldest one
esp_err_t mr_get_data (mr_data_log_t * buffer);

// Gets battery status
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
