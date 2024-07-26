/*
Copyright (c) 2024 Aleksey Kotelnikov <kotelnikov.www@gmail.com>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include "mgos_hx711.h"

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

typedef enum {
  HX711_IDLE,
  HX711_BUSY,
  HX711_SET_GAIN,
  HX711_READ,
  HX711_READ_AVERAGE
} mgos_hx711_state_t;

struct mgos_hx711_reading {
  int32_t data;
  STAILQ_ENTRY(mgos_hx711_reading) next;
};

struct multi_sampling_ctx {
  uint8_t times;
  uint8_t attempt;
  int32_t sum;
};

struct mgos_hx711 {
  const struct mgos_config_hx711 *cfg;
  mgos_hx711_state_t state;
  mgos_timer_id timer;
  int8_t data_gpio;
  int8_t clock_gpio;  
  int8_t gain;
  int32_t poll_period;
  int8_t poll_rate;
  int8_t delay_us;
  int32_t raw_data;
  int32_t offset;
  float scale;
};

struct mgos_hx711 *hx711 = NULL;

// DECLARATION
static bool mgos_hx711_configure_driver();
static uint32_t mgos_hx711_read_raw(void);
static int32_t mgos_hx711_read_data(void);
static void mgos_hx711_dispatcher(void *arg);
static void mgos_hx711_single_reading_cb(void *arg);
static void mgos_hx711_average_reading_cb(void *arg);
static bool mgos_hx711_is_ready(void);
static void mgos_hx711_set_gain(void);

// PUBLIC DEFINITION
void mgos_hx711_power_up(void) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_power_down()"));
  if (hx711 == NULL) return;
  mgos_gpio_write(hx711->clock_gpio, 0);
  mgos_usleep(100);
}

void mgos_hx711_power_down(void) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_power_down()"));
  if (hx711 == NULL) return;
  mgos_gpio_write(hx711->clock_gpio, 1);
  mgos_usleep(100);
}

void mgos_hx711_start_polling(void) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_start_sampling()"));
  if (hx711 == NULL) return;
  if (hx711->poll_period <= 0) {
    LOG(LL_WARN, ("HX711 ->> Polling switched off (check configuration)"));
    return;
  }
  if (hx711->poll_rate > 1) {
    struct multi_sampling_ctx *ctx = (struct multi_sampling_ctx *) calloc(1, sizeof(*ctx));
    ctx->times = hx711->poll_rate;
    ctx->attempt = 0;
    ctx->sum = 0;
    mgos_set_timer(hx711->poll_period, MGOS_TIMER_REPEAT, 
      mgos_hx711_average_reading_cb, ctx);
  } else {
    mgos_set_timer(hx711->poll_period, MGOS_TIMER_REPEAT, 
      mgos_hx711_single_reading_cb, NULL);
  }
}

void mgos_hx711_set_offset(int32_t offset) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_set_offset()"));
  if (hx711 == NULL) return;
  hx711->offset = offset;
}

void mgos_hx711_set_scale(float scale) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_set_scale()"));
  if (hx711 == NULL) return;
  hx711->scale = scale;
}

int32_t mgos_hx711_get_raw_data(void) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_get_raw_data()"));
  if (hx711 == NULL) return 0;
  return hx711->raw_data;
}

int32_t mgos_hx711_get_value(void) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_get_value()"));
  if (hx711 == NULL) return 0;
  return hx711->raw_data - hx711->offset;
}

double mgos_hx711_get_units(void) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_get_units()"));
  if (hx711 == NULL) return 0;
  if (hx711->scale == 1) return 0;
  return mgos_hx711_get_value() / hx711->scale;
}

// PRIVATE DEFINITION
static bool mgos_hx711_configure_driver() {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_configure_driver()"));
  hx711 = (struct mgos_hx711 *) calloc(1, sizeof(*hx711));
  hx711->data_gpio = mgos_sys_config_get_hx711_data_gpio();
  hx711->clock_gpio = mgos_sys_config_get_hx711_clock_gpio();
  hx711->gain = mgos_sys_config_get_hx711_gain();
  hx711->poll_period = mgos_sys_config_get_hx711_poll_period();
  hx711->poll_rate = mgos_sys_config_get_hx711_poll_rate();
  hx711->delay_us = mgos_sys_config_get_hx711_delay_us();
  hx711->raw_data = 0;
  hx711->offset = 0;
  hx711->scale = 1;
  // CHECK CONFIG
  bool config_err = false;
  // CHECK GPIO
  if (hx711->data_gpio < 0) config_err = true;
  if (hx711->clock_gpio < 0) config_err = true;
  if (hx711->data_gpio == hx711->clock_gpio) config_err = true;
  // CHECK GAIN CONFIG
  if (hx711->gain < 0 || hx711->gain > 2) config_err = true;
  // CHECK POLLING
  if (hx711->poll_period < 0) config_err = true;
  if (hx711->poll_rate < 1) config_err = true;
  if (config_err) {
    free(hx711);
    return false;
  }
  // SETUP GPIO
  if (!mgos_gpio_set_mode(hx711->data_gpio, MGOS_GPIO_MODE_INPUT) ||
    !mgos_gpio_set_mode(hx711->clock_gpio, MGOS_GPIO_MODE_OUTPUT)) {
    free(hx711);
    return false;
  };
  mgos_hx711_power_down();
  mgos_hx711_power_up();
  mgos_hx711_set_gain();
  return true;
}

static uint32_t mgos_hx711_read_raw(void) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_read_raw()"));
  uint32_t data = 0;
	// --> Enter critical section
  portENTER_CRITICAL(&mux);
  // Read raw amplifier data
	for (int i = 0; i < 24 ; i++) {
    mgos_gpio_write(hx711->clock_gpio, 1);
    mgos_usleep(hx711->delay_us);
    data |= mgos_gpio_read(hx711->data_gpio) << (23 - i);
    mgos_gpio_write(hx711->clock_gpio, 0);
    mgos_usleep(hx711->delay_us);
	}
  // Set gain + channel for next reading
  for (int i = 0; i <= hx711->gain; i++) {
    mgos_gpio_write(hx711->clock_gpio, 1);
    mgos_usleep(hx711->delay_us);
    mgos_gpio_write(hx711->clock_gpio, 0);
    mgos_usleep(hx711->delay_us);
  }
  portEXIT_CRITICAL(&mux);
	//<-- Exit critical section
  return data;
}

static int32_t mgos_hx711_read_data(void) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_get_raw_data()"));
  uint32_t raw_data = mgos_hx711_read_raw();
  if (raw_data & 0x800000) {
    raw_data |= 0xff000000;
  }
  int32_t data = *((int32_t *)&raw_data);
  return data;
}

static void mgos_hx711_dispatcher(void *arg) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_dispatcher()"));
  switch (hx711->state) {
    case HX711_SET_GAIN: {
      if (!mgos_hx711_is_ready()) return;
      hx711->state = HX711_BUSY;
      mgos_clear_timer(hx711->timer);
      uint32_t set_gain_data = mgos_hx711_read_data();
      LOG(LL_DEBUG, ("HX711 ->> HX711_SET_GAIN_DATA: %d", set_gain_data));
      hx711->state = HX711_IDLE;
      break;
    }
    case HX711_READ: {
      if (!mgos_hx711_is_ready()) return;
      hx711->state = HX711_BUSY;
      mgos_clear_timer(hx711->timer);
      hx711->raw_data = mgos_hx711_read_data();
      hx711->state = HX711_IDLE;
      break;
    }
    case HX711_READ_AVERAGE: {
      if (!mgos_hx711_is_ready()) return;
      hx711->state = HX711_BUSY;
      struct multi_sampling_ctx *ctx = (struct multi_sampling_ctx *) arg;
      if (ctx->attempt <= ctx->times) {
        ctx->attempt++;
        ctx->sum += mgos_hx711_read_data();
        hx711->state = HX711_READ_AVERAGE;
      } else {
        mgos_clear_timer(hx711->timer);
        hx711->raw_data = ctx->sum / ctx->times;
        LOG(LL_DEBUG, ("HX711 ->> RAW_AVG_DATA: %d", hx711->raw_data));
        ctx->attempt = 0;
        ctx->sum = 0;
        hx711->state = HX711_IDLE;
      }
      break;
    }
    case HX711_IDLE:
    case HX711_BUSY:
    default:
      break;
  }
  (void) arg;
}

static void mgos_hx711_single_reading_cb(void *arg) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_single_reading_cb()"));
  if (hx711->state != HX711_IDLE) return;
  hx711->state = HX711_READ;
  hx711->timer = mgos_set_timer(300, MGOS_TIMER_REPEAT, mgos_hx711_dispatcher, NULL);
  (void) arg;
}

static void mgos_hx711_average_reading_cb(void *arg) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_single_reading_cb()"));
  if (hx711->state != HX711_IDLE) return;
  struct multi_sampling_ctx *ctx = (struct multi_sampling_ctx *) arg;
  hx711->state = HX711_READ_AVERAGE;
  hx711->timer = mgos_set_timer(300, MGOS_TIMER_REPEAT, mgos_hx711_dispatcher, ctx);
}

static bool mgos_hx711_is_ready(void) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_is_ready()"));
  bool result = (mgos_gpio_read(hx711->data_gpio) == 0) ? true : false;
  return result;
}

static void mgos_hx711_set_gain(void) {
  LOG(LL_DEBUG, ("HX711 ->> mgos_hx711_set_gain()"));
  hx711->state = HX711_SET_GAIN;
  hx711->timer = mgos_set_timer(50, MGOS_TIMER_REPEAT, mgos_hx711_dispatcher, NULL);
}

// LIBRARY INIT FN
bool mgos_hx711_init(void) {
  if ( !mgos_sys_config_get_hx711_enable() ) {
    LOG(LL_INFO, ("HX711 ->> Library switched off (check configuration)"));
    return true;
  }
  if (!mgos_hx711_configure_driver()) {
    LOG(LL_ERROR, ("HX711 ->> Initializing library unsuccessful (check configuration)"));
    return true;
  }
  LOG(LL_INFO, ("HX711 ->> Initializing library successful"));
  return true;
}