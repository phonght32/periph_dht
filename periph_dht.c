/**
 * @file periph_dht.c
 *
 * ESP-IDF's component implements 1-wire communication to interface with 
 * temperature and humidity sensors such as DHT11, AM2301 (DHT21, DHT22, AM2302,
 * AM2321), SI7021,... The data from sensors is automatically and periodically 
 * updated or refreshed by the user at any time.
 *
 * MIT License
 *
 * Copyright (c) 2023 phonght32
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "periph_dht.h"

/**
 * @macro   Update time in second.
 */
#define TIME_UPDATE_SEC_DEFAULT 		120

/**
 * @macro   Mutex macros.
 */
#define mutex_lock(x)					while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) 				xSemaphoreGive(x)
#define mutex_create()					xSemaphoreCreateMutex()
#define mutex_destroy(x) 				vQueueDelete(x)

/**
 * @macro 	Validate DHT peripheral.
 */
#define VALIDATE_DHT(periph, ret) if(!esp_periph_validate(periph, PERIPH_ID_DHT)) {			\
	ESP_LOGE(TAG, "Invalid PERIPH_ID_DHT");													\
	return ret;																				\
}

/**
 * @macro   DHT check.
 */
#define DHT_CHECK(a, str, action) if(!(a)) {                             	\
    ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);  \
    action;                                                                 \
}

/**
 * @struct 	Peripheral DHT information structure.
 * 
 * @param   type Sensor type.
 * @param  	pin GPIO pin num.
 * @param  	time_update_sec Update time in second.
 * @param   temp Temperature value.
 * @param   humd Humidity value.
 * @param   timer_update The timer handles the update of the sensor's data.
 * @param   lock Mutex.
 * @param   is_started Started status.
 */
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

/**
 * @brief   Module tag that is displayed in ESP_LOG.
 */
static const char *TAG = "PERIPH_DHT";

/**
 * @var   	Global variable stores the DHT peripheral handle structure.
 */
static esp_periph_handle_t g_dht;

/**
 * @func    _dht_init
 */
static esp_err_t _dht_init(esp_periph_handle_t self)
{
	VALIDATE_DHT(self, ESP_FAIL);
	periph_dht_t *periph_dht = esp_periph_get_data(self);
	if (!periph_dht->is_started) {
		periph_dht->is_started = true;
	}

	return ESP_OK;
}

/**
 * @func    _dht_run
 */
static esp_err_t _dht_run(esp_periph_handle_t self, esp_event_iface_msg_t *msg)
{
	return ESP_OK;
}

/**
 * @func 	_dht_destroy
 */
static esp_err_t _dht_destroy(esp_periph_handle_t self)
{
	VALIDATE_DHT(self, ESP_FAIL);
	periph_dht_t *periph_dht = esp_periph_get_data(self);

	xTimerDelete(periph_dht->timer_update, 100);
	mutex_destroy(periph_dht->lock);
	free(periph_dht);

	return ESP_OK;
}

/**
 * @func    _timer_update_cb
 */
static void _timer_update_cb(struct tmrTimerControl *xTimer)
{
	periph_dht_t *periph_dht = esp_periph_get_data(g_dht);

	mutex_lock(periph_dht->lock);
	for (uint8_t i = 0; i < 8; i++) {
		int ret = dht_read_float_data(periph_dht->type, periph_dht->pin, &periph_dht->humd, &periph_dht->temp);
		if (ret == ESP_OK) break;
	}
	mutex_unlock(periph_dht->lock);
}

/**
 * @func    periph_dht_init
 */
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
	periph_dht->timer_update = (TimerHandle_t)xTimerCreate("dht_update_timer",
	                           config->time_update_sec * 1000 / portTICK_RATE_MS,
	                           pdTRUE,
	                           NULL,
	                           _timer_update_cb);
	periph_dht->lock = mutex_create();

	xTimerStart(periph_dht->timer_update, config->time_update_sec * 1000 / portTICK_RATE_MS);

	esp_periph_set_function(periph, _dht_init, _dht_run, _dht_destroy);
	esp_periph_set_data(periph, periph_dht);
	g_dht = periph;

	return periph;
}

/**
 * @func    periph_dht_update_data
 */
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

/**
 * @func    periph_dht_get_data
 */
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