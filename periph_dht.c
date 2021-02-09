#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "periph_dht.h"

#define TIME_UPDATE_SEC_DEFAULT 		120

#define mutex_lock(x)					while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) 				xSemaphoreGive(x)
#define mutex_create()					xSemaphoreCreateMutex()
#define mutex_destroy(x) 				vQueueDelete(x)

#define VALIDATE_DHT(periph, ret) if(!esp_periph_validate(periph, PERIPH_ID_DHT)) {			\
	ESP_LOGE(TAG, "Invalid PERIPH_ID_DHT");													\
	return ret;																					\
}

#define DHT_CHECK(a, str, action) if(!(a)) {                             	\
    ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);  \
    action;                                                                 \
}

typedef struct {
	dht_sensor_type_t 	type;
	gpio_num_t 			pin;
	uint32_t 			time_update_sec;
	float 				temp;
	float 				humd;
	TimerHandle_t 		timer_update;
	SemaphoreHandle_t 	lock;
	bool 				is_started;
} periph_dht_t;

static const char *TAG = "PERIPH_DHT";
static esp_periph_handle_t g_dht;

static esp_err_t _dht_init(esp_periph_handle_t self)
{
	VALIDATE_DHT(self, ESP_FAIL);
	periph_dht_t *periph_dht = esp_periph_get_data(self);
	if (!periph_dht->is_started) {
		periph_dht->is_started = true;
	}

	return ESP_OK;
}

static esp_err_t _dht_run(esp_periph_handle_t self, audio_event_iface_msg_t *msg)
{
	return ESP_OK;
}

static esp_err_t _dht_destroy(esp_periph_handle_t self)
{
	VALIDATE_DHT(self, ESP_FAIL);
	periph_dht_t *periph_dht = esp_periph_get_data(self);

	xTimerDelete(periph_dht->timer_update, 100);
	mutex_destroy(periph_dht->lock);
	free(periph_dht);

	return ESP_OK;
}

static void _timer_update_cb(void *pv)
{
	periph_dht_t *periph_dht = esp_periph_get_data(g_dht);

	mutex_lock(periph_dht->lock);
	for (uint8_t i = 0; i < 8; i++) {
		int ret = dht_read_float_data(periph_dht->type, periph_dht->pin, &periph_dht->humd, &periph_dht->temp);
		if (ret == ESP_OK) break;
	}
	mutex_unlock(periph_dht->lock);
}

esp_periph_handle_t periph_dht_init(periph_dht_cfg_t *config)
{
	DHT_CHECK(config, "error config null", return NULL);

	esp_periph_handle_t periph = esp_periph_create(PERIPH_ID_DHT, config->tag ? config->tag : "periph_dht");
	periph_dht_t *periph_dht = calloc(1, sizeof(periph_dht_t));
	PERIPH_MEM_CHECK(TAG, periph_dht, {free(periph); return NULL;});

	periph_dht->type = config->type;
	periph_dht->pin = config->pin;
	periph_dht->time_update_sec = config->time_update_sec ? config->time_update_sec : TIME_UPDATE_SEC_DEFAULT;
	periph_dht->is_started = false;
	periph_dht->timer_update = xTimerCreate("dht_update_timer", config->time_update_sec * 1000 / portTICK_RATE_MS, pdTRUE, NULL, _timer_update_cb);
	periph_dht->lock = mutex_create();

	xTimerStart(periph_dht->timer_update, config->time_update_sec * 1000 / portTICK_RATE_MS);

	esp_periph_set_function(periph, _dht_init, _dht_run, _dht_destroy);
	esp_periph_set_data(periph, periph_dht);
	g_dht = periph;

	return periph;
}

esp_err_t periph_dht_update_data(esp_periph_handle_t periph)
{
	VALIDATE_DHT(periph, ESP_FAIL);
	periph_dht_t *periph_dht = esp_periph_get_data(periph);

	mutex_lock(periph_dht->lock);
	DHT_CHECK(!dht_read_float_data(periph_dht->type, periph_dht->pin, &periph_dht->humd, &periph_dht->temp), "get data error", {
		mutex_unlock(periph_dht->lock);
		return ESP_FAIL;
	})
	mutex_unlock(periph_dht->lock);

	return ESP_OK;
}

esp_err_t periph_dht_get_data(esp_periph_handle_t periph, float *temp, float *humd)
{
	VALIDATE_DHT(periph, ESP_FAIL);
	periph_dht_t *periph_dht = esp_periph_get_data(periph);

	mutex_lock(periph_dht->lock);
	*temp = periph_dht->temp;
	*humd = periph_dht->humd;
	mutex_unlock(periph_dht->lock);

	return ESP_OK;
}