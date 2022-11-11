#include <stdio.h>
#include "driver/mcpwm.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "bldc.h"

#define BLDC_MCPWM_GROUP   (mcpwm_unit_t)0
#define BLDC_MCPWM_TIMER_U (mcpwm_timer_t)0
#define BLDC_MCPWM_TIMER_V (mcpwm_timer_t)1
#define BLDC_MCPWM_TIMER_W (mcpwm_timer_t)2
#define BLDC_MCPWM_GEN_HIGH MCPWM_GEN_A
#define BLDC_MCPWM_GEN_LOW  MCPWM_GEN_B
#define BLDC_DRV_OVER_CURRENT_FAULT MCPWM_SELECT_F0
#define PULL_UP_EN(x)	if (x >= 0) gpio_pullup_en(x)

static const char *TAG = "bldc";

static inline uint32_t bldc_get_hall_sensor_value(bool ccw, bldc_gpio_t *gpio_conf)
{
	uint32_t hall_val = 
		gpio_get_level(gpio_conf->hall.u) * 4 +
		gpio_get_level(gpio_conf->hall.v) * 2 +
		gpio_get_level(gpio_conf->hall.w) * 1;
	return ccw ? hall_val ^ (0x07) : hall_val;
}

static bool IRAM_ATTR bldc_hall_updated(
		mcpwm_unit_t mcpwm,
		mcpwm_capture_channel_id_t cap_channel,
		const cap_event_data_t *edata,
		void *user_data)
{
	TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
	BaseType_t high_task_wakeup = pdFALSE;
	vTaskNotifyGiveFromISR(task_to_notify, &high_task_wakeup);
	return high_task_wakeup == pdTRUE;
}

static void update_bldc_speed(void *arg)
{
	motor_conf_t *conf = static_cast<motor_conf_t *>(arg);

	if (conf->duty.current < conf->duty.target)
		conf->duty.current += conf->duty.step; /* accelerate */
	else
		conf->duty.current -= conf->duty.step; /* deccelerate */

	if (conf->duty.current > conf->duty.max)
		conf->duty.current = conf->duty.max;
	else if (conf->duty.current < conf->duty.min)
		conf->duty.current = conf->duty.min;
	
	mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH, conf->duty.current);
	mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_LOW, conf->duty.current);
	mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH, conf->duty.current);
	mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_LOW, conf->duty.current);
	mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH, conf->duty.current);
	mcpwm_set_duty(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_LOW, conf->duty.current);
}

// U+V- / A+B-
static void bldc_set_phase_up_vm(void)
{
	mcpwm_set_duty_type(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // U+ = PWM
	mcpwm_deadtime_enable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 3, 3); // U- = _PWM_
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH); // V+ = 0
	mcpwm_set_signal_high(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_LOW); // V- = 1
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH); // W+ = 0
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_LOW);  // W- = 0
}

// W+U- / C+A-
static void bldc_set_phase_wp_um(void)
{
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH); // U+ = 0
	mcpwm_set_signal_high(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_LOW); // U- = 1
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH); // V+ = 0
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_LOW);  // V- = 0
	mcpwm_set_duty_type(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // W+ = PWM
	mcpwm_deadtime_enable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 3, 3); // W- = _PWM_
}

// W+V- / C+B-
static void bldc_set_phase_wp_vm(void)
{
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH); // U+ = 0
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_LOW);  // U- = 0
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH); // V+ = 0
	mcpwm_set_signal_high(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_LOW); // V- = 1
	mcpwm_set_duty_type(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // W+ = PWM
	mcpwm_deadtime_enable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 3, 3); // W- = _PWM_
}

// V+U- / B+A-
static void bldc_set_phase_vp_um(void)
{
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH); // U+ = 0
	mcpwm_set_signal_high(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_LOW); // U- = 1
	mcpwm_set_duty_type(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // V+ = PWM
	mcpwm_deadtime_enable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 3, 3); // V- = _PWM_
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH); // W+ = 0
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_LOW);  // W- = 0
}

// V+W- / B+C-
static void bldc_set_phase_vp_wm(void)
{
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH); // U+ = 0
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_LOW);  // U- = 0
	mcpwm_set_duty_type(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // V+ = PWM
	mcpwm_deadtime_enable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 3, 3); // V- = _PWM_
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH); // W+ = 0
	mcpwm_set_signal_high(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_LOW); // W- = 1
}

// U+W- / A+C-
static void bldc_set_phase_up_wm(void)
{
	mcpwm_set_duty_type(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_MCPWM_GEN_HIGH, MCPWM_DUTY_MODE_0); // U+ = PWM
	mcpwm_deadtime_enable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 3, 3); // U- = _PWM_
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_HIGH); // V+ = 0
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_MCPWM_GEN_LOW); // V- = 0
	mcpwm_deadtime_disable(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W);
	mcpwm_set_signal_low(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_HIGH); // W+ = 0
	mcpwm_set_signal_high(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_MCPWM_GEN_LOW); // W- = 1
}

typedef void (*bldc_hall_phase_action_t)(void);

static const bldc_hall_phase_action_t s_hall_actions[] = {
	bldc_set_phase_vp_wm,
	bldc_set_phase_up_vm,
	bldc_set_phase_up_wm,
	bldc_set_phase_wp_um,
	bldc_set_phase_vp_um,
	bldc_set_phase_wp_vm,
};

static void handler(void *arg)
{
	ESP_LOGI(TAG, "Motor control task started");
	bldc_gpio_t *pin_conf = static_cast<bldc_gpio_t *>(arg);
	//while (1)
		//vTaskDelay(pdMS_TO_TICKS(5000));

	while (1) {
		// The rotation direction is controlled by inverting the hall sensor value
		uint32_t hall_sensor_value = bldc_get_hall_sensor_value(false, pin_conf);
		if (hall_sensor_value < sizeof(s_hall_actions) / sizeof(s_hall_actions[0])) {
			s_hall_actions[hall_sensor_value - 1]();
		} else {
			ESP_LOGE(TAG, "invalid bldc phase, wrong hall sensor value:%d", hall_sensor_value);
			vTaskDelay(pdMS_TO_TICKS(5000));
		}
		//ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}


bldc_gpio_t *create_default_gpio_config() {
	bldc_gpio_t *ret = new bldc_gpio_t();
	gpio_num_t *pin = (gpio_num_t *)ret;
	for (int i = 0; i != sizeof(bldc_gpio_t) / sizeof(gpio_num_t); i++)
		pin[i] = (gpio_num_t)-1;
	
	return ret;
}

esp_err_t BLDC::init()
{
	TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();

	ESP_LOGI(TAG, "Disable gate driver");
	ESP_ERROR_CHECK(gpio_set_direction(pin_conf->ctrl.drv_en, GPIO_MODE_OUTPUT));
	gpio_set_level(pin_conf->ctrl.drv_en, 0);

	ESP_LOGI(TAG, "Setup PWM and Hall GPIO (pull up internally)");
	mcpwm_pin_config_t mcpwm_gpio_config;
	mcpwm_gpio_config.mcpwm0a_out_num = pin_conf->pwm.UH;
	mcpwm_gpio_config.mcpwm0b_out_num = pin_conf->pwm.UL;
	mcpwm_gpio_config.mcpwm1a_out_num = pin_conf->pwm.VH;
	mcpwm_gpio_config.mcpwm1b_out_num = pin_conf->pwm.VL;
	mcpwm_gpio_config.mcpwm2a_out_num = pin_conf->pwm.WH;
	mcpwm_gpio_config.mcpwm2b_out_num = pin_conf->pwm.WL;
	mcpwm_gpio_config.mcpwm_cap0_in_num = pin_conf->hall.u;
	mcpwm_gpio_config.mcpwm_cap1_in_num = pin_conf->hall.v;
	mcpwm_gpio_config.mcpwm_cap2_in_num = pin_conf->hall.w;
	mcpwm_gpio_config.mcpwm_sync0_in_num  = pin_conf->sinc_in[0];
	mcpwm_gpio_config.mcpwm_sync1_in_num  = pin_conf->sinc_in[1];
	mcpwm_gpio_config.mcpwm_sync2_in_num  = pin_conf->sinc_in[2];
	mcpwm_gpio_config.mcpwm_fault0_in_num = pin_conf->ctrl.fault[0];
	mcpwm_gpio_config.mcpwm_fault1_in_num = pin_conf->ctrl.fault[1];
	mcpwm_gpio_config.mcpwm_fault2_in_num = pin_conf->ctrl.fault[2];

	ESP_ERROR_CHECK(mcpwm_set_pin(BLDC_MCPWM_GROUP, &mcpwm_gpio_config));
	// In case there's no pull-up resister for hall sensor on board
	PULL_UP_EN(pin_conf->hall.u);
	PULL_UP_EN(pin_conf->hall.v);
	PULL_UP_EN(pin_conf->hall.w);
	PULL_UP_EN(pin_conf->ctrl.fault[0]);
	PULL_UP_EN(pin_conf->ctrl.fault[1]);
	PULL_UP_EN(pin_conf->ctrl.fault[2]);

	ESP_LOGI(TAG, "Initialize PWM (default to turn off all MOSFET)");
	mcpwm_config_t pwm_config;
	pwm_config.frequency = mot_conf->pwm_frequency;
	pwm_config.cmpr_a = mot_conf->duty.min;
	pwm_config.cmpr_b = mot_conf->duty.min;
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_HAL_GENERATOR_MODE_FORCE_LOW;

	ESP_ERROR_CHECK(mcpwm_init(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, &pwm_config));
	ESP_ERROR_CHECK(mcpwm_init(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, &pwm_config));
	ESP_ERROR_CHECK(mcpwm_init(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, &pwm_config));

	ESP_LOGI(TAG, "Initialize over current fault action");
	ESP_ERROR_CHECK(mcpwm_fault_init(BLDC_MCPWM_GROUP, MCPWM_LOW_LEVEL_TGR, BLDC_DRV_OVER_CURRENT_FAULT));
	ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_U, BLDC_DRV_OVER_CURRENT_FAULT, MCPWM_ACTION_FORCE_LOW, MCPWM_ACTION_FORCE_LOW));
	ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_V, BLDC_DRV_OVER_CURRENT_FAULT, MCPWM_ACTION_FORCE_LOW, MCPWM_ACTION_FORCE_LOW));
	ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(BLDC_MCPWM_GROUP, BLDC_MCPWM_TIMER_W, BLDC_DRV_OVER_CURRENT_FAULT, MCPWM_ACTION_FORCE_LOW, MCPWM_ACTION_FORCE_LOW));

	ESP_LOGI(TAG, "Initialize Hall sensor capture");
	mcpwm_capture_config_t cap_config = {
		.cap_edge = MCPWM_BOTH_EDGE,
		.cap_prescale = 1,
		.capture_cb = bldc_hall_updated,
		.user_data = cur_task,
	};
	ESP_ERROR_CHECK(mcpwm_capture_enable_channel(BLDC_MCPWM_GROUP, MCPWM_SELECT_CAP0, &cap_config));
	ESP_ERROR_CHECK(mcpwm_capture_enable_channel(BLDC_MCPWM_GROUP, MCPWM_SELECT_CAP1, &cap_config));
	ESP_ERROR_CHECK(mcpwm_capture_enable_channel(BLDC_MCPWM_GROUP, MCPWM_SELECT_CAP2, &cap_config));
	ESP_LOGI(TAG, "Please turn on the motor power");
	vTaskDelay(pdMS_TO_TICKS(5000));
	ESP_LOGI(TAG, "Enable gate driver");
	gpio_set_level(pin_conf->ctrl.drv_en, 1);
	ESP_LOGI(TAG, "Changing speed at background");
	esp_timer_create_args_t bldc_timer_args;
	bldc_timer_args.callback = update_bldc_speed;
	bldc_timer_args.name = "bldc_speed";
	bldc_timer_args.arg = mot_conf;
	bldc_timer_args.dispatch_method = ESP_TIMER_TASK;

	ESP_ERROR_CHECK(esp_timer_create(&bldc_timer_args, &bldc_speed_timer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(bldc_speed_timer, 2000000));

	ESP_LOGI(TAG, "Motor started");
	return xTaskCreate(handler, "bldc", 4096, pin_conf, 1, &task_handle);
}

BLDC::~BLDC()
{
	if (task_handle)
		vTaskDelete(task_handle);

	if (is_mot_conf_allocated)
		delete(mot_conf);

	esp_timer_stop(bldc_speed_timer);
	esp_timer_delete(bldc_speed_timer);

	mcpwm_capture_disable_channel(BLDC_MCPWM_GROUP, MCPWM_SELECT_CAP0);
	mcpwm_capture_disable_channel(BLDC_MCPWM_GROUP, MCPWM_SELECT_CAP1);
	mcpwm_capture_disable_channel(BLDC_MCPWM_GROUP, MCPWM_SELECT_CAP2);
	mcpwm_fault_deinit(BLDC_MCPWM_GROUP, BLDC_DRV_OVER_CURRENT_FAULT);

	if (pin_conf) {
		gpio_num_t *pin = (gpio_num_t *)pin_conf;
		for (int i = 0; i != sizeof(pin_conf) / sizeof(gpio_num_t); i++)
			if (pin[i] >= 0)
				gpio_reset_pin(pin[i]);
	}
}
