#include "dispense.h"
#include "ui.h"
#include "zephyr/sys/util.h"
#include "zephyr/sys_clock.h"
#include <lvgl_input_device.h>
#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL

LOG_MODULE_REGISTER(dispense);
static const struct gpio_dt_spec mate_pump =
    GPIO_DT_SPEC_GET(DT_ALIAS(mate_pump), gpios);
static const struct gpio_dt_spec lime_pump =
    GPIO_DT_SPEC_GET(DT_ALIAS(lime_pump), gpios);

static const struct gpio_dt_spec ice_relay =
    GPIO_DT_SPEC_GET(DT_ALIAS(ice_relay), gpios);

static const struct gpio_dt_spec rum_extend =
    GPIO_DT_SPEC_GET(DT_ALIAS(rum_extend), gpios);

static const struct gpio_dt_spec rum_retract =
    GPIO_DT_SPEC_GET(DT_ALIAS(rum_retract), gpios);

static const struct gpio_dt_spec sugar_extend =
    GPIO_DT_SPEC_GET(DT_ALIAS(sugar_extend), gpios);

static const struct gpio_dt_spec sugar_retract =
    GPIO_DT_SPEC_GET(DT_ALIAS(sugar_retract), gpios);

struct s_object s_obj;

const struct smf_state machine_states[SIZEOF_STATES];

static void inital_entry(void *o) {
  struct s_object *s = (struct s_object *)o;

  // smf_set_state(SMF_CTX(&s_obj), &machine_states[]);
}

static void inital_run(void *o) {
  struct s_object *s = (struct s_object *)o;
  printk("Inital");
  smf_set_state(SMF_CTX(&s_obj), &machine_states[MENU]);
}

static void inital_exit(void *o) {
  struct s_object *s = (struct s_object *)o;

  // smf_set_state(SMF_CTX(&s_obj), &machine_states[]);
}
static void menu_entry(void *o) {

  printk("menu");
  struct s_object *s = (struct s_object *)o;
  set_display_home_screen();
  k_event_clear(&s->event, BIT(HOME_SCREEN_DISPENSE_BUTTON_PRESSED));
}
static void menu_run(void *o) {
  struct s_object *s = (struct s_object *)o;

  uint32_t events = k_event_wait(&s->event,
                                 BIT(HOME_SCREEN_DISPENSE_BUTTON_PRESSED) |
                                     BIT(HOME_SCREEN_FEED_LIME_BUTTON_PRESSED) |
                                     BIT(HOME_SCREEN_FEED_MATE_BUTTON_PRESSED) |
                                     BIT(HOME_SCREEN_FEED_RUM_BUTTON_PRESSED),
                                 false, K_NO_WAIT);
  if (events != 0) {
    k_event_clear(&s->event, events);
    LOG_ERR("got it");
    if ((events & BIT(HOME_SCREEN_DISPENSE_BUTTON_PRESSED)) != 0) {
      smf_set_state(SMF_CTX(&s_obj), &machine_states[DISPENSE_ACTIVE]);
    }
    if ((events & BIT(HOME_SCREEN_FEED_LIME_BUTTON_PRESSED)) != 0) {
      s_obj.data.feed_ingredient = LIME;
      smf_set_state(SMF_CTX(&s_obj), &machine_states[FEED]);
    }
    if ((events & BIT(HOME_SCREEN_FEED_MATE_BUTTON_PRESSED)) != 0) {
      s_obj.data.feed_ingredient = MATE;
      smf_set_state(SMF_CTX(&s_obj), &machine_states[FEED]);
    }
    if ((events & BIT(HOME_SCREEN_FEED_RUM_BUTTON_PRESSED)) != 0) {
      s_obj.data.feed_ingredient = RUM;
      smf_set_state(SMF_CTX(&s_obj), &machine_states[FEED]);
    }
  }
}
// START

static void start_run(void *o) { struct s_object *s = (struct s_object *)o; }

struct extend_retract_gpio {
  const struct gpio_dt_spec *extend_gpio;
  const struct gpio_dt_spec *retract_gpio;
};

union ingredient_data {
  const struct gpio_dt_spec *gpio;
  struct extend_retract_gpio extend_retract_gpio;
};

struct ingredient {
  char *name;
  void (*start)(union ingredient_data *);
  void (*stop)(union ingredient_data *);
  union ingredient_data data;
};

void ingredient_start_dispense_gpio(union ingredient_data *data) {
  gpio_pin_set_dt(data->gpio, 1);
}

void ingredient_stop_dispense_gpio(union ingredient_data *data) {
  gpio_pin_set_dt(data->gpio, 0);
}
void ingredient_dispense_extend_gpio(union ingredient_data *data) {

  gpio_pin_set_dt(data->extend_retract_gpio.retract_gpio, 0);
  gpio_pin_set_dt(data->extend_retract_gpio.extend_gpio, 1);
}

void ingredient_dispense_retract_gpio(union ingredient_data *data) {
  gpio_pin_set_dt(data->extend_retract_gpio.extend_gpio, 0);
  gpio_pin_set_dt(data->extend_retract_gpio.retract_gpio, 1);
}

void ingredient_dispense_retract_short_gpio(union ingredient_data *data) {
  gpio_pin_set_dt(data->extend_retract_gpio.extend_gpio, 0);
  gpio_pin_set_dt(data->extend_retract_gpio.retract_gpio, 1);
  k_sleep(K_MSEC(500));
  gpio_pin_set_dt(data->extend_retract_gpio.retract_gpio, 0);
}

struct ingredient ingredients[SIZEOF_INGREDIENTS] = {
    [RUM] =
        {
            .name = "rum",
            .start = &ingredient_dispense_retract_gpio,
            .stop = &ingredient_dispense_extend_gpio,
            .data = {.extend_retract_gpio = {.extend_gpio = &rum_extend,
                                             .retract_gpio = &rum_retract}},
        },
    [ICE] = {.name = "ice",
             .start = &ingredient_start_dispense_gpio,
             .stop = &ingredient_stop_dispense_gpio,
             .data = {.gpio = &ice_relay}},
    [MATE] = {.name = "mate",
              .start = &ingredient_start_dispense_gpio,
              .stop = &ingredient_stop_dispense_gpio,
              .data = {.gpio = &mate_pump}},
    [LIME] = {.name = "lime",
              .start = &ingredient_start_dispense_gpio,
              .stop = &ingredient_stop_dispense_gpio,
              .data = {.gpio = &lime_pump}},
    [SUGAR] = {
        .name = "sugar",
        .start = &ingredient_dispense_extend_gpio,
        .stop = &ingredient_dispense_retract_short_gpio,
        .data = {.extend_retract_gpio = {.extend_gpio = &sugar_extend,
                                         .retract_gpio = &sugar_retract}}}};
struct step {
  struct ingredient *ingredient;
  k_timeout_t time;
};
struct step steps[] = {
    {.ingredient = &ingredients[SUGAR], .time = K_MSEC(500)},
    {.ingredient = &ingredients[SUGAR], .time = K_MSEC(500)},
    {.ingredient = &ingredients[SUGAR], .time = K_MSEC(500)},
    {.ingredient = &ingredients[SUGAR], .time = K_MSEC(500)},
    {
        .ingredient = &ingredients[RUM],
        .time = K_SECONDS(10),
    },
    {.ingredient = &ingredients[ICE], .time = K_SECONDS(6)},
    {.ingredient = &ingredients[MATE], .time = K_SECONDS(7)},
    {.ingredient = &ingredients[LIME], .time = K_SECONDS(2)}};
bool dispense_running = 0;
int dispense_current_ingredient = 0;
static void dispense_entry(void *o) {

  struct s_object *s = (struct s_object *)o;
  dispense_running = 0;
  set_display_btn_screen(false);
  set_text_btn_screen_button("STOP");
  k_event_clear(&s->event, BIT(BTN_SCREEN_BUTTON_PRESSED));
  dispense_current_ingredient = 0;
}

static void dispense_stop_current() {
  if (dispense_running == 1 &&
      dispense_current_ingredient < sizeof(steps) / sizeof(steps[0])) {
    if (steps[dispense_current_ingredient].ingredient->stop != NULL) {
      steps[dispense_current_ingredient].ingredient->stop(
          &steps[dispense_current_ingredient].ingredient->data);
    }
    dispense_running = 0;
    dispense_current_ingredient++;
  }
}
K_TIMER_DEFINE(dispense_timer, NULL, NULL);

static void dispense_run(void *o) {
  struct s_object *s = (struct s_object *)o;
  uint32_t events =
      k_event_wait(&s->event, BIT(BTN_SCREEN_BUTTON_PRESSED), false, K_NO_WAIT);
  if (events != 0) {
    k_event_clear(&s->event, events);
    smf_set_state(SMF_CTX(&s_obj), &machine_states[DISPENSE_ERROR]);
    return;
  }
  if (dispense_running == 0) {
    if (dispense_current_ingredient < sizeof(steps) / sizeof(steps[0])) {
      dispense_running = 1;
      if (steps[dispense_current_ingredient].ingredient->start != NULL) {
        steps[dispense_current_ingredient].ingredient->start(
            &steps[dispense_current_ingredient].ingredient->data);
      }
      if (!K_TIMEOUT_EQ(steps[dispense_current_ingredient].time, K_NO_WAIT)) {
        k_timer_start(&dispense_timer, steps[dispense_current_ingredient].time,
                      K_NO_WAIT);
      } else {
        dispense_stop_current();
      }
    } else {
      LOG_ERR("DONE");
      smf_set_state(SMF_CTX(&s_obj), &machine_states[DISPENSE_DONE]);
      return;
    }
  } else {
    if (k_timer_status_get(&dispense_timer) > 0) {
      dispense_stop_current();
    }
  }
}

static void dispense_exit(void *o) { dispense_stop_current(); }

void dispense_done_entry(void *o) {
  struct s_object *s = (struct s_object *)o;
  set_display_btn_screen(false);
  k_event_clear(&s->event, BIT(BTN_SCREEN_BUTTON_PRESSED));
  set_text_btn_screen_button("Done");
  k_timer_start(&s->timer, K_SECONDS(5), K_NO_WAIT);
}
void dispense_done_run(void *o) {
  struct s_object *s = (struct s_object *)o;
  if (k_timer_status_get(&s->timer) > 0) {
    smf_set_state(SMF_CTX(&s_obj), &machine_states[MENU]);
  }
  char *stats = "Done (timeseconds)";
  sprintf(stats, "Done (%i)", k_timer_remaining_get(&s->timer) / 1000);
  set_text_btn_screen_button(stats);
}
void feed_entry(void *o) {
  set_display_btn_screen(true);
  printk("feed");
}
void feed_run(void *o) {

  struct s_object *s = (struct s_object *)o;
  uint32_t events = k_event_wait(&s->event,
                                 BIT(BTN_SCREEN_BUTTON_PRESSED) |
                                     BIT(BTN_SCREEN_BUTTON_RELEASED) |
                                     BIT(BTN_SCREEN_BACK_BUTTON_PRESSED),
                                 false, K_NO_WAIT);
  if (events != 0) {
    k_event_clear(&s->event, events);
    LOG_ERR("got it");
    if ((events & BIT(BTN_SCREEN_BUTTON_PRESSED)) != 0) {
      ingredients[s->data.feed_ingredient].start(
          &ingredients[s->data.feed_ingredient].data);
    }
    if ((events & BIT(BTN_SCREEN_BUTTON_RELEASED)) != 0) {

      ingredients[s->data.feed_ingredient].stop(
          &ingredients[s->data.feed_ingredient].data);
    }
    if ((events & BIT(BTN_SCREEN_BACK_BUTTON_PRESSED)) != 0) {
      smf_set_state(SMF_CTX(&s_obj), &machine_states[MENU]);
      return;
    }
  }
}
void feed_exit(void *o) {

  struct s_object *s = (struct s_object *)o;
  ingredients[s->data.feed_ingredient].stop(
      &ingredients[s->data.feed_ingredient].data);
}
const struct smf_state machine_states[] = {
    [INITAL] =
        SMF_CREATE_STATE(inital_entry, inital_run, inital_exit, NULL, NULL),
    [MENU] = SMF_CREATE_STATE(menu_entry, menu_run, NULL, NULL, NULL),
    [DISPENSE] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL,
                                  &machine_states[DISPENSE_ACTIVE]),
    [DISPENSE_ACTIVE] =
        SMF_CREATE_STATE(dispense_entry, dispense_run, dispense_exit,
                         &machine_states[DISPENSE], NULL),
    [DISPENSE_DONE] = SMF_CREATE_STATE(dispense_done_entry, dispense_done_run,
                                       NULL, &machine_states[DISPENSE], NULL),
    [DISPENSE_ERROR] = SMF_CREATE_STATE(dispense_done_entry, dispense_done_run,
                                        NULL, &machine_states[DISPENSE], NULL),
    [FEED] = SMF_CREATE_STATE(feed_entry, feed_run, feed_exit, NULL, NULL)};
#define STACK_SIZE 500
#define PRIORITY 5

void dispense_thread_fn() {
  int32_t ret;
  ret = gpio_pin_configure_dt(&mate_pump, GPIO_OUTPUT_INACTIVE);
  ret = gpio_pin_configure_dt(&lime_pump, GPIO_OUTPUT_INACTIVE);

  ret = gpio_pin_configure_dt(&ice_relay, GPIO_OUTPUT_INACTIVE);

  ret = gpio_pin_configure_dt(&rum_extend, GPIO_OUTPUT_INACTIVE);
  ret = gpio_pin_configure_dt(&rum_retract, GPIO_OUTPUT_INACTIVE);
  ret = gpio_pin_configure_dt(&sugar_extend, GPIO_OUTPUT_INACTIVE);
  ret = gpio_pin_configure_dt(&sugar_retract, GPIO_OUTPUT_INACTIVE);
  ui_setup();
  smf_set_initial(SMF_CTX(&s_obj), &machine_states[INITAL]);
  k_event_init(&s_obj.event);
  k_timer_init(&s_obj.timer, NULL, NULL);
  while (1) {

    k_sleep(K_MSEC(15));
    lv_task_handler();
    ret = smf_run_state(SMF_CTX(&s_obj));
    if (ret) {
      break;
    }
  }
};

K_THREAD_DEFINE(dispense_thread_id, 4000, dispense_thread_fn, NULL, NULL, NULL,
                PRIORITY, 0, 0);
