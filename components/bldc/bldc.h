#ifndef __BLDC_H__
#define __BLDC_H__

#include "driver/gpio.h"

class bldc_gpio {
public:
	struct {
		gpio_num_t drv_en;
		gpio_num_t fault;
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
};

class BLDC
{
public:
	BLDC(bldc_gpio& gpio_conf) : pin_conf(gpio_conf) {}
	
	esp_err_t init();

	bldc_gpio& pin_conf;
private:
};

#endif /* __BLDC_H__ */
