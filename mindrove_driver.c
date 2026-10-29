#include <stdio.h>
#include <string.h>

#include "mindrove_driver.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"

// Tag to use for this driver
static const char *TAG = "mindrove driver";

// Flag to assert whether the connection with Mindrove has been initialized or not
static bool mr_netif_done = false;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group = NULL;

// Temporal variable for checking esp_err_t returns
static esp_err_t ret;

// Flag for mindrove connection configuration done
static bool mr_config_for_mr_done = false;

// Wifi status bit
#define WIFI_CONNECTED_BIT BIT0

// Auxiliary definitions
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *ev = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Disconnected, reason=%d", ev->reason);
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Start connection. This function will verify whether the wifi connection has been already established, and, if not, it
// will.
esp_err_t mr_connect(const char *mr_wifi_ssid, const char *mr_wifi_pass, TickType_t ticks_to_wait)
{
    // Create wifi configuration variable
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));

    // Event bits
    EventBits_t bits;
    // Check correct args
    if (mr_wifi_ssid == NULL || mr_wifi_pass == NULL) {
        ESP_LOGE(TAG, "SSID or Password empty.");
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(mr_wifi_ssid) >= 32 || strlen(mr_wifi_pass) >= 64) {
        ESP_LOGE(TAG, "SSID or Password are too long.");
        return ESP_ERR_INVALID_ARG;
    }

    // First, indempotent function
    ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "Error al inicializar NVS.");

    // One-time configuration functions
    // It is checked whether they have already been executed
    if (!mr_netif_done) {
        // Wifi event group create
        if (s_wifi_event_group == NULL) {
            s_wifi_event_group = xEventGroupCreate();
        }

        if (s_wifi_event_group == NULL) {
            ESP_LOGE(TAG, "Probably insufficient heap space for FreeRTOS.");
            return ESP_ERR_NO_MEM;
        }
        // Initialize net infrastructure
        ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Error initializing net infrastructure.");

        // Create the event loop for wifi-related events
        ret = esp_event_loop_create_default();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Error initializing default event loop.");
            return ret;
        }

        // Create event handlers
        static esp_event_handler_instance_t instance_any_id;
        static esp_event_handler_instance_t instance_got_ip;

        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip);

        mr_netif_done = true;
    }

    // Check if user is just trying to reconnect
    if (mr_config_for_mr_done) {
        bits = xEventGroupGetBits(s_wifi_event_group);
        if (!(bits & WIFI_CONNECTED_BIT))
        {
            ESP_LOGI(TAG, "Trying to reconnect with wifi.");
            ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Error starting wifi.");

            bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, ticks_to_wait);

            if (bits & WIFI_CONNECTED_BIT) {
                ESP_LOGI(TAG, "Connected to Mindrove's wifi");
                return ESP_OK;
            } else {
                ESP_LOGE(TAG, "Failed to connect to Mindrove's wifi.");
                return ESP_ERR_WIFI_NOT_CONNECT;
            }
        } else {
            ESP_LOGI(TAG, "Already connected to Mindrove's wifi.");
            return ESP_OK;
        }

    }

    // Check if wifi has been initialized and in what mode
    wifi_mode_t wifi_mode;
    ret = esp_wifi_get_mode(&wifi_mode);

    // Check whether wifi has been initialized and in what mode
    if (ret == ESP_ERR_WIFI_NOT_INIT) {
        // Initialize wifi
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_cfg), TAG, "Error initializing wifi.");

        ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "Error setting wifi's station mode.");

    } else if (wifi_mode == WIFI_MODE_APSTA || wifi_mode == WIFI_MODE_AP) {
        if (wifi_mode == WIFI_MODE_AP)
        {
            esp_netif_create_default_wifi_sta();
        }
        ESP_RETURN_ON_ERROR(esp_wifi_disconnect(), TAG, "Error disconnecting to wifi");
        ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "Error setting wifi on AP/station mode.");

    } else {
        ESP_RETURN_ON_ERROR(esp_wifi_disconnect(), TAG, "Error disconnecting to wifi");
        ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "Error setting wifi's station mode.");
    }

    strcpy((char *)wifi_config.sta.ssid, mr_wifi_ssid);
    strcpy((char *)wifi_config.sta.password, mr_wifi_pass);

    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "Error configuring wifi.");
    mr_config_for_mr_done = true;

    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Error starting wifi.");

    bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, ticks_to_wait);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to Mindrove's wifi");
    } else {
        ESP_LOGE(TAG, "Failed to connect to Mindrove's wifi.");
        return ESP_ERR_WIFI_NOT_CONNECT;
    }

    return ESP_OK;
}

// Starts a task that gets data continuously
esp_err_t mr_get_data_start(void);

// Updates a buffer with all new data, blocks until new data is available
// Inputs:
// data_log_buffer -> cyclic buffer where data will be written
// buffer_size -> quantity of packages the buffer will store
// current_idx -> newest value's index [in/out]
// ticks_to_wait -> ticks to wait for data (suggested > pdMS_TO_TICKS(50))
// Outputs:
// ESP_OK -> buffer has been updated
// ESP_ERR_TIMEOUT -> no data has been found in ticks_to_wait ticks. This may suggest the connection has been lost.
// ESP_ERR_INVALID_STATE -> there has not been a connection with Mindrove Armband or mr_get_data_start() has not been
// called
esp_err_t mr_wait_and_update_buffer(mr_data_log_t *data_log_buffer, uint32_t buffer_size, uint32_t *current_idx,
                                    TickType_t ticks_to_wait);

// Gets battery status
esp_err_t mr_get_batt_status(uint8_t *out_battery_level);

// Terminates connection with mindrove
esp_err_t mr_connection_end(void);
