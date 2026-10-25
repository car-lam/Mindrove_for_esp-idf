/*
This component establishes connection with and samples data from the Mindrove Armband.
*/

#ifndef MINDROVE_DRIVER_H
#define MINDROVE_DRIVER_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

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
esp_err_t mr_connection_init (const char *mr_wifi_ssid, const char *mr_wifi_pass, TickType_t ticks_to_wait);

// Starts a task that gets data continuously
esp_err_t mr_get_data_start (void);

// Updates a buffer with all new data, blocks until new data is available
// Inputs:
// data_log_buffer -> cyclic buffer where data will be written
// buffer_size -> quantity of packages the buffer will store
// current_idx -> newest value's index [in/out]
// ticks_to_wait -> ticks to wait for data (suggested > pdMS_TO_TICKS(50))
// Outputs:
// ESP_OK -> buffer has been updated
// ESP_ERR_TIMEOUT -> no data has been found in ticks_to_wait ticks. This may suggest the connection has been lost.
// ESP_ERR_INVALID_STATE -> there has not been a connection with Mindrove Armband or mr_get_data_start() has not been called
esp_err_t mr_wait_and_update_buffer(mr_data_log_t *data_log_buffer, uint32_t buffer_size, uint32_t *current_idx, TickType_t ticks_to_wait);

// Gets battery status
esp_err_t mr_get_batt_status (uint8_t *out_battery_level);

// Terminates connection with mindrove
esp_err_t mr_connection_end (void);

#endif
