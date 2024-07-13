/*
 * Copyright (c) 2018 Peter Bigot Consulting, LLC
 * Copyright (c) 2018 Aapo Vienamo
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2020 ZedBlox Ltd.
 * Copyright (c) 2021 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/util_macro.h"
#include <errno.h>
#include <sys/_stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(m5stm32f0, CONFIG_GPIO_LOG_LEVEL);

#include <zephyr/drivers/gpio/gpio_utils.h>

/* Max to select all pins supported on the device. */

/** Cache of the output configuration and data of the pins. */
struct m5stm32f0_pin_state {
  uint8_t mode;
  uint16_t value;
};

struct m5stm32f0_irq_state {
  uint8_t rising;
  uint8_t falling;
};

/** Runtime driver data */
struct m5stm32f0_drv_data {
  /* gpio_driver_data needs to be first */
  struct gpio_driver_data common;
  struct m5stm32f0_pin_state pin_state[8];
  struct k_sem lock;
  struct gpio_callback gpio_cb;
  struct k_work work;
  struct m5stm32f0_irq_state irq_state;
  const struct device *dev;
  /* user ISR cb */
  sys_slist_t cb;
};

/** Configuration data */
struct m5stm32f0_config {
  /* gpio_driver_config needs to be first */
  struct gpio_driver_config common;
  struct i2c_dt_spec i2c;
  const struct gpio_dt_spec gpio_int;
  bool interrupt_enabled;
  int interrupt_mask;
  int input_latch;
};

/**
 * @brief Gets the state of input pins of the m5stm32f0 I/O Port and
 * stores in driver data struct.
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 * @retval Negative value for error code.
 */
static int update_input(const struct device *dev) {
  const struct m5stm32f0_config *cfg = dev->config;
  struct m5stm32f0_drv_data *drv_data = dev->data;
  uint8_t input_states;
  int rc = 0;
  for (uint8_t i = 0; i < 8; i++) {
    i2c_reg_read_byte_dt(&cfg->i2c, 0x0 + i, &drv_data->pin_state[i].mode);
  }

  return rc;
}

static int gpio_m5stm32f0_config(const struct device *dev, gpio_pin_t pin,
                                 gpio_flags_t flags) {
  const struct m5stm32f0_config *cfg = dev->config;
  struct m5stm32f0_drv_data *drv_data = dev->data;
  if (pin > 8) {
    return -ENOTSUP;
  }
  struct m5stm32f0_pin_state *pins = &drv_data->pin_state[pin];
  int rc = 0;
  bool data_first = false;

  /* Can't do I2C bus operations from an ISR */
  if (k_is_in_isr()) {
    return -EWOULDBLOCK;
  }

  /* Single Ended lines (Open drain and open source) not supported */
  if ((flags & GPIO_SINGLE_ENDED) != 0) {
    return -ENOTSUP;
  }

  /* The m5stm32f0 has no internal pull up support */
  if (((flags & GPIO_PULL_UP) != 0) || ((flags & GPIO_PULL_DOWN) != 0)) {
    return -ENOTSUP;
  }

  /* Simultaneous input & output mode not supported */
  if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0)) {
    return -ENOTSUP;
  }

  k_sem_take(&drv_data->lock, K_FOREVER);

  /* Ensure either Output or Input is specified */
  if ((flags & GPIO_OUTPUT) != 0) {
    pins->mode = 1;
    if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
      pins->value = 0;
      data_first = true;
    } else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
      pins->value = 1;
      data_first = true;
    }
  } else if ((flags & GPIO_INPUT) != 0) {
    pins->mode = 0;
  } else {
    rc = -ENOTSUP;
    goto out;
  }

  /* Set output values */
  if (data_first) {
    rc = i2c_reg_write_byte_dt(&cfg->i2c, 0x10 + pin, pins->value);
  }

  if (rc == 0) {
    /* Set pin directions */
    rc = i2c_reg_write_byte_dt(&cfg->i2c, 0x0 + pin, pins->mode);
  }

  if (rc == 0) {
    /* Refresh input status */
    rc = update_input(dev);
  }

out:
  k_sem_give(&drv_data->lock);
  return rc;
}

static int gpio_m5stm32f0_port_read(const struct device *dev,
                                    gpio_port_value_t *value) {
  const struct m5stm32f0_config *cfg = dev->config;
  struct m5stm32f0_drv_data *drv_data = dev->data;
  uint8_t input_pin_data;
  int rc = 0;

  /* Can't do I2C bus operations from an ISR */
  if (k_is_in_isr()) {
    return -EWOULDBLOCK;
  }

  k_sem_take(&drv_data->lock, K_FOREVER);
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t pin_value = 0;
    struct m5stm32f0_pin_state *pins = &drv_data->pin_state[i];
    if (pins->mode == 1) {
      rc |= i2c_reg_read_byte_dt(&cfg->i2c, 0x10 + i, &pin_value);
    } else if (pins->mode == 0) {
      rc |= i2c_reg_read_byte_dt(&cfg->i2c, 0x10 + i, &pin_value);
    }
    pins->value = pin_value;
    gpio_port_value_t current_value = *value;
    uint16_t current_value_int = current_value;
    current_value = WRITE_BIT(current_value_int, i, pin_value);
    *value = current_value;
  }
  /* Read Input Register */

  k_sem_give(&drv_data->lock);
  return rc;
}

static int gpio_m5stm32f0_port_write(const struct device *dev,
                                     gpio_port_pins_t mask,
                                     gpio_port_value_t value,
                                     gpio_port_value_t toggle) {
  const struct m5stm32f0_config *cfg = dev->config;
  struct m5stm32f0_drv_data *drv_data = dev->data;
  int rc;
  uint8_t orig_out;
  uint8_t out;

  /* Can't do I2C bus operations from an ISR */
  if (k_is_in_isr()) {
    return -EWOULDBLOCK;
  }

  k_sem_take(&drv_data->lock, K_FOREVER);

  for (uint8_t i = 0; i < 8; i++) {
    struct m5stm32f0_pin_state *pins = &drv_data->pin_state[i];
    if (((BIT(i) & mask) || (BIT(i) & toggle)) && pins->mode == 1) {
      if (BIT(i) & value) {
        pins->value = 1;
      } else {
        pins->value = 0;
      }
      if (BIT(i) & toggle) {
        pins->value = ~pins->value;
      }

      rc = i2c_reg_write_byte_dt(&cfg->i2c, 0x10 + i, pins->value);
    }
  }

  if (rc == 0) {
  }

  k_sem_give(&drv_data->lock);

  return rc;
}

static int gpio_m5stm32f0_port_set_masked(const struct device *dev,
                                          gpio_port_pins_t mask,
                                          gpio_port_value_t value) {
  return gpio_m5stm32f0_port_write(dev, mask, value, 0);
}

static int gpio_m5stm32f0_port_set_bits(const struct device *dev,
                                        gpio_port_pins_t pins) {
  return gpio_m5stm32f0_port_write(dev, pins, pins, 0);
}

static int gpio_m5stm32f0_port_clear_bits(const struct device *dev,
                                          gpio_port_pins_t pins) {
  return gpio_m5stm32f0_port_write(dev, pins, 0, 0);
}

static int gpio_m5stm32f0_port_toggle_bits(const struct device *dev,
                                           gpio_port_pins_t pins) {
  return gpio_m5stm32f0_port_write(dev, 0, 0, pins);
}

static int gpio_m5stm32f0_manage_callback(const struct device *dev,
                                          struct gpio_callback *callback,
                                          bool set) {
  struct m5stm32f0_drv_data *data = dev->data;

  return gpio_manage_callback(&data->cb, callback, set);
}

/**
 * @brief Initialization function of m5stm32f0
 *
 * This sets initial input/ output configuration and output states.
 * The interrupt is configured if this is enabled.
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int gpio_m5stm32f0_init(const struct device *dev) {
  const struct m5stm32f0_config *cfg = dev->config;
  struct m5stm32f0_drv_data *drv_data = dev->data;
  int rc = 0;

  if (!device_is_ready(cfg->i2c.bus)) {
    LOG_ERR("I2C bus device not found");
    goto out;
  }

  /* Do an initial read, this clears the interrupt pin and sets
   * up the initial value of the pin state input data.
   */
  rc = update_input(dev);
  if (rc) {
    goto out;
  }

out:
  if (rc) {
    ;
    LOG_ERR("%s init failed: %d", dev->name, rc);
  } else {
    LOG_INF("%s init ok", dev->name);
  }
  return rc;
}

static const struct gpio_driver_api api_table = {
    .pin_configure = gpio_m5stm32f0_config,
    .port_get_raw = gpio_m5stm32f0_port_read,
    .port_set_masked_raw = gpio_m5stm32f0_port_set_masked,
    .port_set_bits_raw = gpio_m5stm32f0_port_set_bits,
    .port_clear_bits_raw = gpio_m5stm32f0_port_clear_bits,
    .port_toggle_bits = gpio_m5stm32f0_port_toggle_bits,
    .manage_callback = gpio_m5stm32f0_manage_callback,
};

#define GPIO_m5stm32f0_INIT(n)                                                 \
  static const struct m5stm32f0_config m5stm32f0_cfg_##n = {                   \
      .i2c = I2C_DT_SPEC_INST_GET(n),                                          \
      .common =                                                                \
          {                                                                    \
              .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),             \
          },                                                                   \
  };                                                                           \
                                                                               \
  static struct m5stm32f0_drv_data m5stm32f0_drvdata_##n = {                   \
      .lock = Z_SEM_INITIALIZER(m5stm32f0_drvdata_##n.lock, 1, 1),             \
      .pin_state = {{}, {}, {}, {}, {}, {}, {}, {}}                           \
  }                                                                            \
  ;                                                                            \
  DEVICE_DT_INST_DEFINE(n, gpio_m5stm32f0_init, NULL, &m5stm32f0_drvdata_##n,  \
                        &m5stm32f0_cfg_##n, POST_KERNEL,                       \
                        60, &api_table);

#define DT_DRV_COMPAT m5_stm32f0
DT_INST_FOREACH_STATUS_OKAY(GPIO_m5stm32f0_INIT)
