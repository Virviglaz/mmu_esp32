#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bldc.h"
#include "esp_log.h"

extern "C" {
	void app_main();
}

void app_main(void)
{
	bldc_gpio_t *motor_gpio = create_default_gpio_config();
	motor_gpio->ctrl.drv_en = GPIO_NUM_2;
	motor_gpio->pwm.UH = GPIO_NUM_12;
	motor_gpio->pwm.UL = GPIO_NUM_13;
	motor_gpio->pwm.VH = GPIO_NUM_14;
	motor_gpio->pwm.VL = GPIO_NUM_15;
	motor_gpio->pwm.WH = GPIO_NUM_16;
	motor_gpio->pwm.WL = GPIO_NUM_17;
	motor_gpio->hall.u = GPIO_NUM_18;
	motor_gpio->hall.v = GPIO_NUM_19;
	motor_gpio->hall.w = GPIO_NUM_21;

	BLDC *motor = new BLDC(motor_gpio);

	motor->init();

	int i = 0;
	while (1) {
		//printf("[%d] Hello world!\n", i);
		ESP_LOGI("main", "[%d] Hello world!\n", i);
		i++;
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}
