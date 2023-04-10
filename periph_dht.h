/**
 * @file periph_dht.h
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

#ifndef _PERIPH_DHT_H_
#define _PERIPH_DHT_H_

#include "esp_err.h"
#include "driver/gpio.h"

#include "esp_peripherals.h"
#include "dht.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct  Peripheral DHT configuration structure.
 * 
 * @param   tag Module tag that is displayed in ESP_LOG.
 * @param  	type Sensor type.
 * @param  	pin GPIO pin num.
 * @param  	time_update_sec Update time in second.
 */
typedef struct {
	const char 			*tag;
	dht_sensor_type_t 	type;
	gpio_num_t 			pin;
	uint32_t 			time_update_sec;
} periph_dht_cfg_t;

/**
 * @brief   Initialize DHT peripheral.
 * 
 * @param   config Pointer references to the configuration structure.
 *
 * @return  
 * 		- Pointer to the DHT peripheral handle structure.
 * 		- NULL: Failed. 
 */
esp_periph_handle_t periph_dht_init(periph_dht_cfg_t *config);

/**
 * @brief   Update sensor's data.
 * 
 * @param   periph DHT peripheral handle structure.
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_dht_update_data(esp_periph_handle_t periph);

/**
 * @brief   Get sensor's data.
 * 
 * @param   periph DHT peripheral handle structure.
 * @param 	temp Pointer references to temperature.  
 * @param 	humd Pointer references to humidity.  
 *
 * @return  
 *      - ESP_OK:       Success.
 *      - Others:       Failed. 
 */
esp_err_t periph_dht_get_data(esp_periph_handle_t periph, float *temp, float *humd);


#ifdef __cplusplus
}
#endif

#endif /* _PERIPH_DHT_H_ */