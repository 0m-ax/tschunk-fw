#include "ui.h"
#include "core/lv_obj.h"
#include "dispense.h"
#include "zephyr/smf.h"
#include <lvgl_input_device.h>
#include <sys/_stdint.h>
#include <zephyr/kernel.h>
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>

#include <stdlib.h>
#include <zephyr/drivers/display.h>
LOG_MODULE_REGISTER(ui);
K_SEM_DEFINE(ui_ready, 0, 1);
// INPUT_CALLBACK_DEFINE(NULL, input_cb);
static void scroll_event_cb(lv_event_t *e) {
  // return;
  lv_obj_t *cont = lv_event_get_target(e);

  lv_area_t cont_a;
  lv_obj_get_coords(cont, &cont_a);
  lv_coord_t cont_y_center = cont_a.y1 + lv_area_get_height(&cont_a) / 2;

  lv_coord_t r = lv_obj_get_height(cont) * 9 / 10;
  uint32_t i;
  uint32_t child_cnt = lv_obj_get_child_cnt(cont);
  for (i = 0; i < child_cnt; i++) {
    lv_obj_t *child = lv_obj_get_child(cont, i);
    lv_area_t child_a;
    lv_obj_get_coords(child, &child_a);

    lv_coord_t child_y_center = child_a.y1 + lv_area_get_height(&child_a) / 2;

    lv_coord_t diff_y = child_y_center - cont_y_center;
    diff_y = LV_ABS(diff_y);

    /*Get the x of diff_y on a circle.*/
    lv_coord_t x;
    /*If diff_y is out of the circle use the last point of the circle (the
     * radius)*/
    if (diff_y >= r) {
      x = r;
    } else {
      /*Use Pythagoras theorem to get x from radius and y*/
      uint32_t x_sqr = r * r - diff_y * diff_y;
      lv_sqrt_res_t res;
      lv_sqrt(x_sqr, &res, 0x8000); /*Use lvgl's built in sqrt root function*/
      x = r - res.i;
    }

    /*Translate the item by the calculated X coordinate*/
    lv_obj_set_style_translate_x(child, x, 0);

    /*Use some opacity with larger translations*/
    lv_opa_t opa = lv_map(x, 0, r, LV_OPA_TRANSP, LV_OPA_COVER);
    lv_obj_set_style_opa(child, LV_OPA_COVER - opa, 0);
  }
}

/**
 * Translate the object as they scroll
 */
lv_obj_t *lv_create_menu(lv_obj_t *parent) {
  lv_obj_t *cont = lv_obj_create(parent);
  lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
  lv_obj_center(cont);
  lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_add_event_cb(cont, scroll_event_cb, LV_EVENT_SCROLL, NULL);
  lv_obj_set_scroll_dir(cont, LV_DIR_VER);
  lv_obj_set_scroll_snap_y(cont, LV_SCROLL_SNAP_CENTER);
  lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
  return cont;
}
void lv_init_menu(lv_obj_t *menu) {

  /*Update the buttons position manually for first*/
  lv_event_send(menu, LV_EVENT_SCROLL, NULL);

  /*Be sure the fist button is in the middle*/
  lv_obj_scroll_to_view(lv_obj_get_child(menu, 0), LV_ANIM_OFF);
}

extern struct s_object s_obj;

extern const struct smf_state machine_states[SIZEOF_STATES];
static void button_pressed_cb(lv_event_t *e) {

  enum DISPENSE_EVENTS *target =
      (enum DISPENSE_EVENTS *)lv_event_get_user_data(e);
  enum DISPENSE_EVENTS event = *target;
  k_event_set(&s_obj.event, BIT(event));
  // lv_obj_t **target = (lv_obj_t **)lv_event_get_user_data(e);
  // lv_scr_load(*target);
}
static uint32_t count = 1;

struct home_screen_buttons {
  enum DISPENSE_EVENTS event;
  char *title;
} home_screen_buttons[4] = {
    {.event = HOME_SCREEN_DISPENSE_BUTTON_PRESSED, .title = "START"},
    {.event = HOME_SCREEN_FEED_LIME_BUTTON_PRESSED, .title = "Feed Lime"},
    {.event = HOME_SCREEN_FEED_MATE_BUTTON_PRESSED, .title = "Feed Mate"},
    {.event = HOME_SCREEN_FEED_RUM_BUTTON_PRESSED, .title = "Feed Rum"}};

static enum DISPENSE_EVENTS btn_screen_button_pressed =
    BTN_SCREEN_BUTTON_PRESSED;
static enum DISPENSE_EVENTS btn_screen_back_button_pressed =
    BTN_SCREEN_BACK_BUTTON_PRESSED;
static enum DISPENSE_EVENTS btn_screen_button_released =
    BTN_SCREEN_BUTTON_RELEASED;

#include <zephyr/shell/shell.h>
int cmd_send_event(const struct shell *sh, size_t argc, char **argv) {
  int cnt;
  uint32_t value = atoi(argv[1]);
  LOG_ERR("sending event %i", value);
  k_event_set(&s_obj.event, BIT(value));
  return 0;
}
SHELL_COND_CMD_ARG_REGISTER(1, send_event, NULL, "SEND an event",
                            cmd_send_event, 2, 0);

lv_obj_t *home_screen;
void set_display_home_screen() { lv_scr_load(home_screen); }

lv_obj_t *btn_screen_back_btn;
void show_btn_screen_back_button() {
  lv_obj_clear_flag(btn_screen_back_btn, LV_OBJ_FLAG_HIDDEN);
}
void hide_btn_screen_back_button() {
  lv_obj_add_flag(btn_screen_back_btn, LV_OBJ_FLAG_HIDDEN);
}
lv_obj_t *btn_screen;
void set_display_btn_screen(bool back_button) {
  lv_scr_load(btn_screen);
  if (back_button) {
    show_btn_screen_back_button();
  } else {
    hide_btn_screen_back_button();
  }
}
lv_obj_t *btn_screen_btn_label;
void set_text_btn_screen_button(char *text) {
  lv_label_set_text(btn_screen_btn_label, text);
}

int ui_setup(void) {
  const struct device *display_dev;
  display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
  if (!device_is_ready(display_dev)) {
    LOG_ERR("Device not ready, aborting test");
    return 0;
  }

  const struct device *lvgl_encoder =
      DEVICE_DT_GET_ANY(zephyr_lvgl_encoder_input);
  if (!device_is_ready(lvgl_encoder)) {
    LOG_ERR("encoder not ready");
  }

  home_screen = lv_obj_create(NULL);
  lv_obj_t *menu = lv_create_menu(home_screen);
  for (int i = 0;
       i < sizeof(home_screen_buttons) / sizeof(home_screen_buttons[0]); i++) {
    // lv_obj_t *item = lv_obj_create(menu);

    // lv_obj_set_size(item, lv_pct(100), lv_pct(80));
    lv_obj_t *btn = lv_obj_create(menu);
    lv_obj_set_style_bg_color(btn, lv_palette_lighten(LV_PALETTE_BLUE, 1), 0);
    lv_obj_set_style_radius(btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_size(btn, lv_pct(75), lv_pct(75));

    lv_obj_add_event_cb(btn, button_pressed_cb, LV_EVENT_LONG_PRESSED,
                        &home_screen_buttons[i].event);
    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, home_screen_buttons[i].title);

    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
  }
  lv_init_menu(menu);

  btn_screen = lv_obj_create(NULL);

  lv_obj_t *btn_screen_btns = lv_obj_create(btn_screen);

  lv_obj_t *btn_screen_btn = lv_btn_create(btn_screen_btns);
  lv_obj_add_event_cb(btn_screen_btn, button_pressed_cb, LV_EVENT_PRESSED,
                      &btn_screen_button_pressed);
  lv_obj_add_event_cb(btn_screen_btn, button_pressed_cb, LV_EVENT_RELEASED,
                      &btn_screen_button_released);
  btn_screen_btn_label = lv_label_create(btn_screen_btn);
  lv_label_set_text(btn_screen_btn_label, "Hello world!");

  // lv_obj_align(btn_screen_btn, LV_ALIGN_TOP_MID, 0, 0);
  btn_screen_back_btn = lv_btn_create(btn_screen_btns);
  lv_obj_add_event_cb(btn_screen_back_btn, button_pressed_cb, LV_EVENT_PRESSED,
                      &btn_screen_back_button_pressed);
  lv_obj_t *btn_screen_back_btn_label = lv_label_create(btn_screen_back_btn);
  lv_label_set_text(btn_screen_back_btn_label, "BACK");

  // lv_obj_align(btn_screen_back_btn, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_align(btn_screen_btns, LV_ALIGN_CENTER, 0, 0);
  // arc = lv_arc_create(lv_scr_act());
  // lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);
  // lv_obj_set_size(arc, 200, 200);
  //
  // arc_group = lv_group_create();
  // lv_group_add_obj(arc_group, arc);
  // lv_indev_set_group(lvgl_input_get_indev(lvgl_encoder), arc_group);
  //
  // lv_example_scroll_6();

  display_blanking_off(display_dev);
}

// K_THREAD_DEFINE(my_tid, 4000, ui_setup, NULL, NULL, NULL, 15, 0, 0);
