
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/smf.h>

enum DISPENSE_STATES {
  INITAL,
  MENU,
  FEED,
  DISPENSE,
  DISPENSE_ACTIVE,
  DISPENSE_DONE,
  DISPENSE_ERROR,
  ERROR,
  SIZEOF_STATES
};
enum DISPENSE_EVENTS {
  // HOME screen
  HOME_SCREEN_DISPENSE_BUTTON_PRESSED,
  HOME_SCREEN_FEED_LIME_BUTTON_PRESSED,
  HOME_SCREEN_FEED_MATE_BUTTON_PRESSED,
  HOME_SCREEN_FEED_RUM_BUTTON_PRESSED,
  // BTN SCREEN
  BTN_SCREEN_BUTTON_PRESSED,
  BTN_SCREEN_BUTTON_RELEASED,
  BTN_SCREEN_BACK_BUTTON_PRESSED,
  SIZEOF_EVENTS
};

union event_data {};

enum INGREDIENTS { RUM, ICE, MATE, LIME, SUGAR, SIZEOF_INGREDIENTS };
struct s_object {
  struct smf_ctx ctx;
  struct k_event event;
  union event_data event_data[SIZEOF_EVENTS];
  struct data {
    enum INGREDIENTS feed_ingredient;
  } data;
  struct k_timer timer;
};
