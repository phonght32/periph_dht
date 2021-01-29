#ifndef _PERIPH_DHT_H_
#define _PERIPH_DHT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_peripherals.h"
#include "driver/gpio.h"
#include "dht.h"

typedef struct {
	const char 			*tag;
	dht_sensor_type_t 	type;
	gpio_num_t 			pin;
	uint32_t 			time_update_sec;
} periph_dht_cfg_t;

esp_periph_handle_t periph_dht_init(periph_dht_cfg_t *config);
esp_err_t periph_dht_update_data(esp_periph_handle_t periph);
esp_err_t periph_dht_get_data(esp_periph_handle_t periph, float *temp, float *humd);

#ifdef __cplusplus
}
#endif

#endif /* _PERIPH_DHT_H_ */