/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lv_api_map.h"
#include "misc/lv_area.h"
#include "widgets/lv_label.h"
#include <stdio.h>
#include <zephyr/kernel.h>

int maina(void) {
  while (1) {
    k_msleep(1000);
    printf("hello");
  }
  return 0;
}

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/gpio.h>
static void input_cb(struct input_event *evt) {
  printf("Key press");

  printf("code: %i, value: %i type: %i", evt->code, evt->value, evt->type);
}

INPUT_CALLBACK_DEFINE(NULL, input_cb);

#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/display.h>

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>


static const struct gpio_dt_spec mate_pump =
    GPIO_DT_SPEC_GET(DT_ALIAS(mate_pump), gpios);
static const struct gpio_dt_spec lime_pump =
    GPIO_DT_SPEC_GET(DT_ALIAS(lime_pump), gpios);

static const struct gpio_dt_spec ice_relay =
    GPIO_DT_SPEC_GET(DT_ALIAS(ice_relay), gpios);

int main(void) {
  int32_t grey_scale_sleep;
  const struct device *display_dev;
  int ret;
  ret = gpio_pin_configure_dt(&mate_pump, GPIO_OUTPUT_INACTIVE);
  ret = gpio_pin_configure_dt(&lime_pump, GPIO_OUTPUT_INACTIVE);

  ret = gpio_pin_configure_dt(&ice_relay, GPIO_OUTPUT_INACTIVE);
  k_msleep(2000);
  display_dev =  DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
  if (!device_is_ready(display_dev)) {
    printk("diasplay not ready");
    return 0;
  }
  printk("hello");
  lv_obj_t *hello_world_obj;
  hello_world_obj = lv_label_create(lv_scr_act());
  lv_label_set_text(hello_world_obj, "hello");
  lv_obj_align(hello_world_obj, LV_ALIGN_CENTER, 0, 0);
  lv_task_handler();
  display_blanking_off(display_dev);
  while (1) {
    lv_task_handler();
    k_sleep(K_SECONDS(1000));
  }
}
