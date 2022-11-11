#ifndef __BLDC_H__
#define __BLDC_H__

#include "driver/gpio.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdint.h>

typedef struct {
	struct {
		gpio_num_t drv_en;
		gpio_num_t fault[3];
		gpio_num_t overcurrent;
	} ctrl;

	struct {
		gpio_num_t UH;
		gpio_num_t UL;
		gpio_num_t VH;
		gpio_num_t VL;
		gpio_num_t WH;
		gpio_num_t WL;
	} pwm;

	struct {
		gpio_num_t u;
		gpio_num_t v;
		gpio_num_t w;
	} hall;

	gpio_num_t sinc_in[3];
} bldc_gpio_t;

bldc_gpio_t *create_default_gpio_config();

typedef struct {
	uint32_t pwm_frequency;
	struct {
		float min;
		float current;
		float step;
		float max;
		float target;
	} duty;
} motor_conf_t;

class BLDC
{
public:
	BLDC(bldc_gpio_t *gpio_conf, motor_conf_t *motor_conf) : 
		pin_conf(gpio_conf),
		mot_conf(motor_conf) {}

	BLDC(bldc_gpio_t *gpio_conf) : pin_conf(gpio_conf) {
		mot_conf = new motor_conf_t();
		mot_conf->pwm_frequency = 14400;
		mot_conf->duty.min = 10;
		mot_conf->duty.current = 10;
		mot_conf->duty.max = 80;
		mot_conf->duty.step = 5;
		is_mot_conf_allocated = true;
	}

	~BLDC();

	esp_err_t init();

	motor_conf_t *get_motor_config() {
		return mot_conf;
	}

	void set_throttle(float throttle) {
		mot_conf->duty.target = throttle;
	}

	float get_throttle() {
		return mot_conf->duty.current;
	}

private:
	bldc_gpio_t *pin_conf = nullptr;
	motor_conf_t *mot_conf = nullptr;
	bool is_mot_conf_allocated = false;
	TaskHandle_t task_handle = nullptr;
	esp_timer_handle_t bldc_speed_timer;
};

#endif /* __BLDC_H__ */
