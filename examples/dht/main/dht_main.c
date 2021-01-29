#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "periph_dht.h"

static const char *TAG = "APP_MAIN";
static esp_periph_handle_t dht_handle = NULL;

static esp_err_t periph_event_handle(audio_event_iface_msg_t *event, void *context)
{
    return ESP_OK;
}

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("PERIPH_DHT", ESP_LOG_INFO);

    esp_periph_config_t config = {
        .event_handle = periph_event_handle,
        .max_parallel_connections = 9,
    };
    esp_periph_init(&config);

    periph_dht_cfg_t periph_dht_cfg = {
        .type = DHT_TYPE_AM2301,
        .pin = 32,
        .time_update_sec = 1,
    };
    dht_handle = periph_dht_init(&periph_dht_cfg);

    float temp, humd;

    while (1) {
        periph_dht_get_data(dht_handle, &temp, &humd);
        ESP_LOGI(TAG, "temp: %f, humd: %f", temp, humd);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
