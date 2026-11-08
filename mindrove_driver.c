#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "mindrove_driver.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
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

// Wifi state found by mr_connect() when the wifi had already been initialized by someone else.
// mr_connection_end() restores it.
static bool mr_prev_mode_saved = false;
static wifi_mode_t mr_prev_mode = WIFI_MODE_NULL;
static bool mr_prev_sta_cfg_saved = false;
static wifi_config_t mr_prev_sta_cfg;

// UDP receive timeout. Short so the UDP task notices a stop request quickly.
#define MR_UDP_RCV_TIMEOUT_MS 300
// Consecutive receive timeouts before warning that no data arrives (~5 s)
#define MR_UDP_NO_DATA_WARN_COUNT (5000 / MR_UDP_RCV_TIMEOUT_MS)

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

// Stop tasks and events request
static volatile bool mr_stop_requested = false;

// Declare event handlers
static esp_event_handler_instance_t instance_any_id;
static esp_event_handler_instance_t instance_got_ip;

static esp_netif_t *mr_netif = NULL;

static bool mr_this_driver_initialized_wifi = false;

// Task-finished bits
#define MR_UDP_TASK_DONE_BIT    BIT0
#define MR_DECODE_TASK_DONE_BIT BIT1
#define MR_ALL_TASKS_DONE_BITS  (MR_UDP_TASK_DONE_BIT | MR_DECODE_TASK_DONE_BIT)

static EventGroupHandle_t mr_tasks_event_group = NULL;

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
        if (!mr_disconnect_requested) {
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

    esp_err_t ret;

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

        ESP_RETURN_ON_ERROR(
            esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id),
            TAG, "Error registering wifi event handler.");

        ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL,
                                                  &instance_got_ip);
        if (ret != ESP_OK) {
            // Undo the first registration so a later retry does not register it twice
            esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id);
            ESP_LOGE(TAG, "Error registering IP event handler.");
            return ret;
        }

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
        if (mr_netif == NULL) {
            ESP_LOGE(TAG, "Net infraestructure for station could not be created.");
            return ESP_ERR_WIFI_INIT_STATE;
        }
        wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_cfg), TAG, "Error initializing wifi.");
        mr_this_driver_initialized_wifi = true;

        ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "Error setting wifi's station mode.");

    } else if (ret != ESP_OK) {
        // wifi_mode is not valid here
        ESP_LOGE(TAG, "Error reading wifi mode.");
        return ret;

    } else {
        // Wifi belongs to someone else: save its state so mr_connection_end() can restore it
        mr_prev_mode = wifi_mode;
        mr_prev_mode_saved = true;

        if (wifi_mode == WIFI_MODE_STA || wifi_mode == WIFI_MODE_APSTA) {
            if (esp_wifi_get_config(WIFI_IF_STA, &mr_prev_sta_cfg) == ESP_OK) {
                mr_prev_sta_cfg_saved = true;
            }
        }

        if (wifi_mode == WIFI_MODE_AP) {
            // There is no station interface yet: the driver creates it (and destroys it in mr_connection_end())
            mr_netif = esp_netif_create_default_wifi_sta();
            if (mr_netif == NULL) {
                ESP_LOGE(TAG, "Net infraestructure for station could not be created.");
                return ESP_ERR_NO_MEM;
            }
        }

        // Not started yet is fine: there is nothing to disconnect from
        ret = esp_wifi_disconnect();
        if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_STARTED) {
            ESP_LOGE(TAG, "Error disconnecting from wifi.");
            return ret;
        }

        wifi_mode_t new_mode = (wifi_mode == WIFI_MODE_AP || wifi_mode == WIFI_MODE_APSTA) ? WIFI_MODE_APSTA
                                                                                           : WIFI_MODE_STA;
        ESP_RETURN_ON_ERROR(esp_wifi_set_mode(new_mode), TAG, "Error setting wifi mode.");
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
static TaskHandle_t udp_client_handle = NULL;

static void udp_client_task(void *pvParameters)
{
    uint8_t rx_buffer[128];
    struct sockaddr_in mr_sockaddr;
    int sock;
    memset(&mr_sockaddr, 0, sizeof(mr_sockaddr));
    mr_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    mr_sockaddr.sin_family = AF_INET;
    mr_sockaddr.sin_port = htons(MR_UDP_PORT);

    while (!mr_stop_requested) {

        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

        if (sock < 0) {
            ESP_LOGE(TAG, "Socket could not be created: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = MR_UDP_RCV_TIMEOUT_MS * 1000;
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
        int n_timeouts = 0;

        while (!mr_stop_requested) {
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
            if (len < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // Warn once every ~5 s without data, not on every timeout
                    n_timeouts++;
                    if (n_timeouts >= MR_UDP_NO_DATA_WARN_COUNT) {
                        ESP_LOGW(TAG, "5 seconds passed without receiving data. Waiting.");
                        n_timeouts = 0;
                    }
                } else {
                    ESP_LOGE(TAG, "Socket error: errno %d", errno);
                    break;
                }
            }

            else {
                n_timeouts = 0;

                if (len != MR_PKG_SIZE) {
                    ESP_LOGI(TAG, "Data is invalid.");
                } else if (xQueueSend(mr_to_decode_queue, rx_buffer, pdMS_TO_TICKS(10)) != pdPASS) {
                    ESP_LOGE(TAG, "The queue is full.");
                }
            }
        }

        shutdown(sock, 0);
        close(sock);

        if (!mr_stop_requested) {
            ESP_LOGW(TAG, "Restarting socket...");
            // Avoid a tight loop if the error persists
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Single exit point: always tell mr_acquisition_teardown() that this task is done,
    // whether it was asked to stop or it failed on its own
    xEventGroupSetBits(mr_tasks_event_group, MR_UDP_TASK_DONE_BIT);
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
static TaskHandle_t mr_decode_and_update_buffer_handle = NULL;
static uint32_t mr_buffer_idx_head = 0;
static bool mr_buffer_full = false;

static void mr_decode_and_update_buffer(void *pvParameters)
{
    mr_data_log_t log;
    uint8_t buf[128];

    while (!mr_stop_requested) {
        // The 50 ms timeout lets the task check the stop request regularly
        if (xQueueReceive(mr_to_decode_queue, buf, pdMS_TO_TICKS(50)) && mr_decode(buf, &log) == ESP_OK) {
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

    // Single exit point
    xEventGroupSetBits(mr_tasks_event_group, MR_DECODE_TASK_DONE_BIT);
    vTaskDelete(NULL);
}

// Stops the acquisition tasks (if running) and frees the resources they share.
// Safe to call more than once. If a task does not stop in time nothing is freed,
// because that task may still be using the queue, mutex or buffer.
static esp_err_t mr_acquisition_teardown(void)
{
    // Wait only for the tasks that actually exist
    EventBits_t wait_bits = 0;
    if (udp_client_handle != NULL) {
        wait_bits |= MR_UDP_TASK_DONE_BIT;
    }
    if (mr_decode_and_update_buffer_handle != NULL) {
        wait_bits |= MR_DECODE_TASK_DONE_BIT;
    }

    if (wait_bits != 0) {
        mr_stop_requested = true;

        EventBits_t bits = xEventGroupWaitBits(mr_tasks_event_group, wait_bits, pdTRUE, pdTRUE,
                                               pdMS_TO_TICKS(2000)); // > SO_RCVTIMEO

        if ((bits & wait_bits) != wait_bits) {
            ESP_LOGE(TAG, "Tasks did not stop in time, resources not freed.");
            return ESP_ERR_TIMEOUT;
        }

        udp_client_handle = NULL;
        mr_decode_and_update_buffer_handle = NULL;
    }

    // Free in reverse order of creation. The tasks' event group is kept (created once).
    if (mr_to_decode_queue != NULL) {
        vQueueDelete(mr_to_decode_queue);
        mr_to_decode_queue = NULL;
    }
    if (mr_buffer_mutex != NULL) {
        vSemaphoreDelete(mr_buffer_mutex);
        mr_buffer_mutex = NULL;
    }
    if (mr_buffer != NULL) {
        free(mr_buffer);
        mr_buffer = NULL;
    }

    mr_buffer_idx_head = 0;
    mr_buffer_full = false;
    mr_buffer_size = 0;
    mr_battery = 0;

    return ESP_OK;
}

// Starts a task that gets data continuously
// This function calls mr_get_data once the buffer is full so the user only works with complete buffers
esp_err_t mr_get_data_start(size_t buffer_size)
{
    esp_err_t err = ESP_OK;

    // 0. Validations. Nothing has been created yet, so plain returns are fine here.
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Wifi has not been initialized, run mr_connect() first.");
        return ESP_ERR_WIFI_NOT_CONNECT;
    }

    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "The deviced is not connected yet, run mr_connect() first.");
        return ESP_ERR_WIFI_NOT_CONNECT;
    }

    if (buffer_size == 0) {
        ESP_LOGE(TAG, "Buffer size must be greater than 0.");
        return ESP_ERR_INVALID_ARG;
    }

    if (udp_client_handle != NULL || mr_decode_and_update_buffer_handle != NULL) {
        ESP_LOGE(TAG, "Data acquisition already running, run mr_connection_end() first.");
        return ESP_ERR_INVALID_STATE;
    }

    // 1. Set the internal buffer that will store the data
    mr_buffer = heap_caps_malloc(buffer_size * sizeof(*mr_buffer), MALLOC_CAP_8BIT);
    if (mr_buffer == NULL) {
        ESP_LOGE(TAG, "Error creating the internal buffer, it may not be sufficient memory on the heap.");
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    mr_buffer_mutex = xSemaphoreCreateMutex();
    if (mr_buffer_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create the buffer mutex.");
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    // Create the queue in which the two tasks will comunicate
    mr_to_decode_queue = xQueueCreate(4, sizeof(uint8_t) * 128);
    if (mr_to_decode_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create the queue to store data to be decoded.");
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    if (mr_tasks_event_group == NULL) {
        mr_tasks_event_group = xEventGroupCreate();
        if (mr_tasks_event_group == NULL) {
            ESP_LOGE(TAG, "Failed to create the tasks event group.");
            err = ESP_ERR_NO_MEM;
            goto cleanup;
        }
    }

    // 2. Reset state before the tasks start using it
    mr_buffer_size = buffer_size;
    mr_buffer_idx_head = 0;
    mr_buffer_full = false;
    xEventGroupClearBits(mr_tasks_event_group, MR_ALL_TASKS_DONE_BITS);
    mr_stop_requested = false;

    // 3. Create the tasks
    if (xTaskCreate(udp_client_task, "udp_client_task", 4096, NULL, 5, &udp_client_handle) != pdPASS) {
        udp_client_handle = NULL;
        ESP_LOGE(TAG, "Failed to create the UDP client task.");
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    if (xTaskCreate(mr_decode_and_update_buffer, "decode_and_update_buffer", 4096, NULL, 4,
                    &mr_decode_and_update_buffer_handle) != pdPASS) {
        mr_decode_and_update_buffer_handle = NULL;
        ESP_LOGE(TAG, "Failed to create the decoding task.");
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    return ESP_OK;

cleanup:
    // Stops whatever task was created and frees whatever is not NULL
    mr_acquisition_teardown();
    return err;
}

// Writes the stored data onto a buffer
// The first sample (idx 0) is the oldest one
esp_err_t mr_get_data(mr_data_log_t *buffer)
{
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Buffer is NULL.");
        return ESP_ERR_INVALID_ARG;
    }
    if (mr_buffer_mutex == NULL) {
        ESP_LOGE(TAG, "Run mr_get_data_start() first.");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(mr_buffer_mutex, pdMS_TO_TICKS(10))) {
        if (!mr_buffer_full) {
            xSemaphoreGive(mr_buffer_mutex);
            ESP_LOGW(TAG, "Buffer not full.");
            return ESP_ERR_NOT_FINISHED;
        }

        memcpy(buffer, (mr_buffer + mr_buffer_idx_head), (mr_buffer_size - mr_buffer_idx_head) * sizeof(mr_data_log_t));

        memcpy(buffer + mr_buffer_size - mr_buffer_idx_head, (mr_buffer), (mr_buffer_idx_head) * sizeof(mr_data_log_t));

        xSemaphoreGive(mr_buffer_mutex);

        ESP_LOGI(TAG, "Buffer up-to-date");
        return ESP_OK;
    } else {
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

// Stores the first error found, so teardown can go on and still report what failed
static void mr_keep_first_err(esp_err_t *first, esp_err_t e, const char *what)
{
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "%s: %s", what, esp_err_to_name(e));
        if (*first == ESP_OK) {
            *first = e;
        }
    }
}

// Terminates connection with mindrove
esp_err_t mr_connection_end(void)
{
    // 1. Stop tasks and free acquisition resources.
    // If a task is still alive, leave the network untouched: it may still be using the socket.
    // The user can call mr_connection_end() again.
    esp_err_t err = mr_acquisition_teardown();
    if (err != ESP_OK) {
        return err;
    }

    // 2. Nothing to do if mr_connect() never got this far (or mr_connection_end() already ran)
    if (!mr_netif_done) {
        return ESP_OK;
    }

    esp_err_t e;

    // 3. Disconnect. The flag keeps the event handler from reconnecting.
    mr_disconnect_requested = true;
    e = esp_wifi_disconnect();
    if (e != ESP_ERR_WIFI_NOT_STARTED && e != ESP_ERR_WIFI_NOT_INIT) {
        mr_keep_first_err(&err, e, "Error disconnecting from wifi");
    }

    // 4. Remove the event handlers. From here on nobody clears the connected bit, so do it by hand.
    mr_keep_first_err(&err, esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id),
                      "Error unregistering wifi event handler");
    mr_keep_first_err(&err, esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip),
                      "Error unregistering IP event handler");
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    // 5. Leave the wifi as it was before mr_connect()
    if (mr_this_driver_initialized_wifi) {
        // The driver brought the wifi up, so it brings it down. Order matters: stop -> deinit -> netif.
        e = esp_wifi_stop();
        if (e != ESP_ERR_WIFI_NOT_INIT) {
            mr_keep_first_err(&err, e, "Error stopping wifi");
        }
        e = esp_wifi_deinit();
        if (e != ESP_ERR_WIFI_NOT_INIT) {
            mr_keep_first_err(&err, e, "Error deinitializing wifi");
        }
    } else if (mr_prev_mode_saved) {
        // The wifi belongs to someone else: give back its station config and mode, do not stop it.
        // Its owner has to call esp_wifi_connect() again if it wants its previous network back.
        if (mr_prev_sta_cfg_saved) {
            mr_keep_first_err(&err, esp_wifi_set_config(WIFI_IF_STA, &mr_prev_sta_cfg),
                              "Error restoring previous station config");
        }
        mr_keep_first_err(&err, esp_wifi_set_mode(mr_prev_mode), "Error restoring previous wifi mode");
    }

    // mr_netif is only non-NULL when this driver created it (wifi not initialized, or AP mode)
    if (mr_netif != NULL) {
        esp_netif_destroy_default_wifi(mr_netif);
        mr_netif = NULL;
    }

    // 6. Back to the initial state, so the next mr_connect() starts from scratch
    mr_config_for_mr_done = false;
    mr_this_driver_initialized_wifi = false;
    mr_prev_mode_saved = false;
    mr_prev_sta_cfg_saved = false;
    mr_netif_done = false;

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Connection with Mindrove ended.");
    }
    return err;
}
