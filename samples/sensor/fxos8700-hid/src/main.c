/*
 * Copyright (c) 2018 qianfan Zhao
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2019 Intel Corp
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main);

#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
static const struct gpio_dt_spec sw0_gpio = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
#endif

/* If second button exists, use it as right-click. */
#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
static const struct gpio_dt_spec sw1_gpio = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
#endif

static const struct gpio_dt_spec led_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);


static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);

static uint32_t def_val[4];
static volatile uint8_t status[4];
static K_SEM_DEFINE(sem, 0, 1);	/* starts off "not available" */
static struct gpio_callback gpio_callbacks[4];

#define MOUSE_BTN_REPORT_POS	0
#define MOUSE_X_REPORT_POS	1
#define MOUSE_Y_REPORT_POS	2

#define MOUSE_BTN_LEFT		BIT(0)
#define MOUSE_BTN_RIGHT		BIT(1)
#define MOUSE_BTN_MIDDLE	BIT(2)


static void left_button(const struct device *gpio, struct gpio_callback *cb,
			uint32_t pins)
{
	uint32_t cur_val;
	uint8_t state = status[MOUSE_BTN_REPORT_POS];

	cur_val = gpio_pin_get(gpio, pins);
	if (def_val[0] != cur_val) {
		state |= MOUSE_BTN_LEFT;
	} else {
		state &= ~MOUSE_BTN_LEFT;
	}

	if (status[MOUSE_BTN_REPORT_POS] != state) {
		status[MOUSE_BTN_REPORT_POS] = state;
		k_sem_give(&sem);
	}
}

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
static void right_button(const struct device *gpio, struct gpio_callback *cb,
			 uint32_t pins)

{
	uint32_t cur_val;
	uint8_t state = status[MOUSE_BTN_REPORT_POS];

	cur_val = gpio_pin_get(gpio, pins);
	if (def_val[0] != cur_val) {
		state |= MOUSE_BTN_RIGHT;
	} else {
		state &= ~MOUSE_BTN_RIGHT;
	}

	if (status[MOUSE_BTN_REPORT_POS] != state) {
		status[MOUSE_BTN_REPORT_POS] = state;
		k_sem_give(&sem);
	}
}
#endif

int callbacks_configure(const struct gpio_dt_spec *gpio,
			gpio_callback_handler_t handler,
			struct gpio_callback *callback, uint32_t *val)
{
	int ret;

	if (!device_is_ready(gpio->port)) {
		LOG_ERR("%s: device not ready.", gpio->port->name);
		return -ENODEV;
	}

	gpio_pin_configure_dt(gpio, GPIO_INPUT);

	ret = gpio_pin_get_dt(gpio);
	if (ret < 0) {
		return ret;
	}

	*val = (uint32_t)ret;

	gpio_init_callback(callback, handler, BIT(gpio->pin));
	gpio_add_callback(gpio->port, callback);
	gpio_pin_interrupt_configure_dt(gpio, GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}

static bool read_accel(const struct device *dev)
{
	struct sensor_value val[3];
	int ret;

	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, val);
	if (ret < 0) {
		LOG_ERR("Cannot read sensor channels");
		return false;
	}

	/* For printing double we need to use printf with
	 * printf("%10.6f\n", sensor_value_to_double(x));
	 */
	LOG_DBG("int parts: X %d Y %d", val[0].val1, val[1].val1);

	/* TODO: Add proper calculations */

	if (val[0].val1 != 0) {
		status[MOUSE_X_REPORT_POS] = val[0].val1 * 4;
	}

	if (val[1].val1 != 0) {
		status[MOUSE_Y_REPORT_POS] = val[1].val1 * 4;
	}

	if (val[0].val1 != 0 || val[1].val1 != 0) {
		return true;
	} else {
		return false;
	}
}

static void trigger_handler(const struct device *dev,
			    const struct sensor_trigger *tr)
{
	ARG_UNUSED(tr);

	/* Always fetch the sample to clear the data ready interrupt in the
	 * sensor.
	 */
	if (sensor_sample_fetch(dev)) {
		LOG_ERR("sensor_sample_fetch failed");
		return;
	}

	if (read_accel(dev)) {
		k_sem_give(&sem);
	}
}

int main(void)
{
	int ret;
	uint8_t report[4] = { 0x00 };
	const struct device *accel_dev, *hid_dev;

	if (!gpio_is_ready_dt(&led_gpio)) {
		LOG_ERR("%s: device not ready.", led_gpio.port->name);
		return 0;
	}

	hid_dev = device_get_binding("HID_0");
	if (hid_dev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return 0;
	}

	gpio_pin_configure_dt(&led_gpio, GPIO_OUTPUT);

	if (callbacks_configure(&sw0_gpio, &left_button, &gpio_callbacks[0], &def_val[0])) {
		LOG_ERR("Failed configuring left button callback.");
		return 0;
	}

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
	if (callbacks_configure(&sw1_gpio, &right_button, &gpio_callbacks[1], &def_val[1])) {
		LOG_ERR("Failed configuring right button callback.");
		return 0;
	}
#endif

	accel_dev = DEVICE_DT_GET_ONE(nxp_fxos8700);
	if (!device_is_ready(accel_dev)) {
		LOG_ERR("%s: device not ready.", accel_dev->name);
		return 0;
	}

	struct sensor_value attr = {
		.val1 = 6,
		.val2 = 250000,
	};

	if (sensor_attr_set(accel_dev, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &attr)) {
		LOG_ERR("Could not set sampling frequency");
		return 0;
	}

	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	if (sensor_trigger_set(accel_dev, &trig, trigger_handler)) {
		LOG_ERR("Could not set trigger");
		return 0;
	}

	usb_hid_register_device(hid_dev, hid_report_desc,
				sizeof(hid_report_desc), NULL);

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	usb_hid_init(hid_dev);

	while (true) {
		k_sem_take(&sem, K_FOREVER);

		report[MOUSE_BTN_REPORT_POS] = status[MOUSE_BTN_REPORT_POS];
		report[MOUSE_X_REPORT_POS] = status[MOUSE_X_REPORT_POS];
		status[MOUSE_X_REPORT_POS] = 0U;
		report[MOUSE_Y_REPORT_POS] = status[MOUSE_Y_REPORT_POS];
		status[MOUSE_Y_REPORT_POS] = 0U;
		hid_int_ep_write(hid_dev, report, sizeof(report), NULL);

		/* Toggle LED on sent report */
		gpio_pin_toggle_dt(&led_gpio);
	}
}
