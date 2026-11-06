#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "mindrove_driver.h"

#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include <errno.h> // IWYU pragma: keep
#include <lwip/netdb.h>

// Wifi status bit
#define WIFI_CONNECTED_BIT BIT0

// Mindrove package size
#define MR_PKG_SIZE 75

// Puerto del Mindrove
#define MR_UDP_PORT 4210

// Decoding constants
#define MR_EMG_CNT_TO_UV 0.045f
#define MR_ACC_CNT_TO_G (1.0f / 16384.0f)
#define MR_GYR_CNT_TO_DPS (1.0f / 65.5f)

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

// Internal buffer to store the data
static mr_data_log_t *mr_buffer = NULL;

// Mutex for mr_buff
static SemaphoreHandle_t mr_buffer_mutex;

// Size of mr_buffer
static size_t mr_buffer_size;

// Internal queue to decode data
static QueueHandle_t mr_to_decode_queue = NULL;

// Mindrove battery
static uint8_t mr_battery;

// Disconnect from wifi request
static volatile bool mr_disconnect_requested = false;

// Create event handlers
static esp_event_handler_instance_t instance_any_id;
static esp_event_handler_instance_t instance_got_ip;

static esp_netif_t *mr_netif = NULL;

static bool mr_this_driver_initialized_wifi = false;

// Auxiliary definitions
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        mr_disconnect_requested = false;
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *ev = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Disconnected, reason=%d", ev->reason);
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        if (!mr_disconnect_requested)
        {
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
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

        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip);

        mr_netif_done = true;
    }

    // Check if user is just trying to reconnect
    if (mr_config_for_mr_done) {
        bits = xEventGroupGetBits(s_wifi_event_group);
        if (!(bits & WIFI_CONNECTED_BIT)) {
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
        mr_netif = esp_netif_create_default_wifi_sta();
        if (mr_netif == NULL)
        {
            ESP_LOGE(TAG, "Net infraestructure for station could not be created.");
            return ESP_ERR_WIFI_INIT_STATE;
        }
        wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_cfg), TAG, "Error initializing wifi.");
         mr_this_driver_initialized_wifi = true;

        ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "Error setting wifi's station mode.");

    } else if (wifi_mode == WIFI_MODE_APSTA || wifi_mode == WIFI_MODE_AP) {
        if (wifi_mode == WIFI_MODE_AP) {
            mr_netif = esp_netif_create_default_wifi_sta();
            if (mr_netif == NULL)
            {
                ESP_LOGE(TAG, "Net infraestructure for station could not be created.");
                return ESP_ERR_WIFI_INIT_STATE;
            }
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

// Task Handle
TaskHandle_t udp_client_handle;

static void udp_client_task(void *pvParameters)
{
    uint8_t rx_buffer[128];
    struct sockaddr_in mr_sockaddr;
    int sock;
    memset(&mr_sockaddr, 0, sizeof(mr_sockaddr));
    mr_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    mr_sockaddr.sin_family = AF_INET;
    mr_sockaddr.sin_port = htons(MR_UDP_PORT);

    while (1) {

        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

        if (sock < 0) {
            ESP_LOGE(TAG, "Socket could not be created.");
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        // Bind
        int err = bind(sock, (const struct sockaddr *)&mr_sockaddr, sizeof(mr_sockaddr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            close(sock);
            break;
        }

        // Loop for receiving data and writing it onto the buffer
        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);

        while (1) {
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
            if (len < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    ESP_LOGW(TAG, "5 seconds passed without receiving data. Waiting.");
                } else {
                    ESP_LOGE(TAG, "Socket error.");
                    break;
                }
            }

            else if (len != MR_PKG_SIZE) {
                ESP_LOGI(TAG, "Data is invalid.");
            }

            else {
                if (xQueueSend(mr_to_decode_queue, rx_buffer, pdMS_TO_TICKS(10)) != pdPASS) {
                    ESP_LOGE(TAG, "The queue is full.");
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }

    vTaskDelete(NULL);
}

static inline int32_t int24_be(const uint8_t *bytes)
{
    int32_t val = ((int32_t)bytes[0] << 8 * 2) | ((int32_t)bytes[1] << 8) | bytes[2];
    return (val & 0x800000) ? val - 0x1000000 : val;
}

static inline int16_t int16_le(const uint8_t *bytes)
{
    int16_t val = ((int16_t)bytes[1] << 8) | bytes[0];
    return val;
}

static inline uint32_t uint32_le(const uint8_t *bytes)
{
    uint32_t val = ((uint32_t)bytes[3] << 8 * 3) | ((uint32_t)bytes[2] << 8 * 2) | ((uint32_t)bytes[1] << 8) |
                   ((uint32_t)bytes[0]);
    return val;
}

static esp_err_t mr_decode(const uint8_t buf[128], mr_data_log_t *log)
{
    // Verificar primeros 2 bytes de verificación
    if (buf[0] == 0x02 && buf[1] == 0x00) {
        // EMG data
        for (int i = 0; i < MR_N_SAMP_PER_PACK_EMG; i++) {
            for (int j = 0; j < MR_N_EMG_CHANNELS; j++) {
                log->emg[i][j] = int24_be(buf + 2 + i * 24 + j * 3) * MR_EMG_CNT_TO_UV;
            }
        }
        // Battery status
        mr_battery = buf[50];

        for (int i = 0; i < MR_ACCELEROMETER_CHANNELS; i++) {
            log->acc[i] = int16_le(buf + 55 + i * 2) * MR_ACC_CNT_TO_G;
            log->gyro[i] = int16_le(buf + 61 + i * 2) * MR_GYR_CNT_TO_DPS;
        }

        log->idx = uint32_le(buf + 67);

        return ESP_OK;

    } else {
        ESP_LOGI(TAG, "Incorrect header");
        return ESP_ERR_INVALID_ARG;
    }
}

// Task handle
TaskHandle_t mr_decode_and_update_buffer_handle;
static uint32_t mr_buffer_idx_head = 0;
static bool mr_buffer_full = false;

static void mr_decode_and_update_buffer(void *pvParameters)
{
    while (1) {

        mr_data_log_t log;
        uint8_t buf[128];
        esp_err_t err;

        while (1) {
            if (xQueueReceive(mr_to_decode_queue, buf, pdMS_TO_TICKS(50))) {
                err = mr_decode(buf, &log);
                if (err == ESP_OK) {
                    // Put data onto the internal buffer
                    if (xSemaphoreTake(mr_buffer_mutex, pdMS_TO_TICKS(5))) {
                        mr_buffer[mr_buffer_idx_head] = log;
                        mr_buffer_idx_head++;
                        if (mr_buffer_idx_head == mr_buffer_size) {
                            mr_buffer_idx_head = 0;
                            mr_buffer_full = true;
                        }
                        xSemaphoreGive(mr_buffer_mutex);
                    }
                }
            }
        }
    }
}

// Starts a task that gets data continuously
// This function calls mr_get_data once the buffer is full so the user only works with complete buffers
esp_err_t mr_get_data_start(size_t buffer_size)
{
    // 0. Check if connection has been stablished
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Wifi has not been initialized, run mr_connect() first.");
        return ESP_ERR_WIFI_NOT_CONNECT;
    }

    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "The deviced is not connected yet, run mr_connect() first.");
        return ESP_ERR_WIFI_NOT_CONNECT;
    }

    // 1. Set the internal buffer that will store the data
    mr_buffer_size = buffer_size;
    mr_buffer = heap_caps_malloc(mr_buffer_size * sizeof(*mr_buffer), MALLOC_CAP_8BIT);

    if (mr_buffer == NULL) {
        ESP_LOGE(TAG, "Error creating the internal buffer, it may not be sufficient memory on the heap.");
        return ESP_ERR_NO_MEM;
    }

    mr_buffer_mutex = xSemaphoreCreateMutex();

    // Create the queue in which the two tasks will comunicate
    mr_to_decode_queue = xQueueCreate(4, sizeof(uint8_t) * 128);
    if (mr_to_decode_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create the queue to store data to be decoded.");
        return ESP_ERR_NO_MEM;
    }

    xTaskCreate(udp_client_task, "udp_client_task", 4096, NULL, 5, &udp_client_handle);
    xTaskCreate(mr_decode_and_update_buffer, "decode_and_update_buffer", 4096, NULL, 4,
                &mr_decode_and_update_buffer_handle);

    return ESP_OK;
}

// Writes the stored data onto a buffer
// The first sample (idx 0) is the oldest one
esp_err_t mr_get_data(mr_data_log_t *buffer){
    if (buffer == NULL)
    {
        ESP_LOGE(TAG, "Buffer is NULL.");
        return ESP_ERR_INVALID_ARG;
    }
    if (mr_buffer_mutex == NULL)
    {
        ESP_LOGE(TAG, "Run mr_get_data_start() first.");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(mr_buffer_mutex, pdMS_TO_TICKS(10)))
    {
        if (!mr_buffer_full)
        {
            xSemaphoreGive(mr_buffer_mutex);
            ESP_LOGW(TAG, "Buffer not full.");
            return ESP_ERR_NOT_FINISHED;
        }

        memcpy(buffer,
            (mr_buffer + mr_buffer_idx_head),
            (mr_buffer_size - mr_buffer_idx_head) * sizeof(mr_data_log_t));

        memcpy(buffer + mr_buffer_size - mr_buffer_idx_head,
            (mr_buffer),
            (mr_buffer_idx_head) * sizeof(mr_data_log_t));

        xSemaphoreGive(mr_buffer_mutex);

        ESP_LOGI(TAG, "Buffer up-to-date");
        return ESP_OK;
    }
    else
    {
        ESP_LOGW(TAG, "Buffer could not be updated. Possibly the mutex was taken.");
        return ESP_ERR_TIMEOUT;
    }
}

// Gets battery status
esp_err_t mr_get_batt_status(uint8_t *out_battery_level)
{
    *out_battery_level = mr_battery;
    return ESP_OK;
}

// Terminates connection with mindrove
esp_err_t mr_connection_end(void)
{
    mr_disconnect_requested = true;
    // Delete wifi connection
    ESP_RETURN_ON_ERROR(esp_wifi_disconnect(), TAG, "Error disconnecting from wifi.");
    ESP_RETURN_ON_ERROR(esp_wifi_stop(), TAG, "Error stoping wifi.");
    return ESP_OK;
}
