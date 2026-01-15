#include <pico/error.h>
#include <pico/stdio.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

// ---------------- iBUS definitions ----------------
#define IBUS_BAUD 115200
#define IBUS_FRAME_LEN 32
#define IBUS_HEADER0 0x20
#define IBUS_HEADER1 0x40
#define IBUS_CH_COUNT 14

// Failsafe: if no valid frame for this long =>oopsie, safe outputs so we don't
// run through the fence
#define FAILSAFE_TIMEOUT_US 100000 // 100 ms

// TODO: Check these with the scope. I don't trust the receiver.
//  Typical RC range
#define CH_MIN_US 1000
#define CH_MAX_US 2000
#define CH_CENTER_US 1500

// ---------------- UART pins ----------------
#define IBUS_UART uart0
#define IBUS_UART_RX_PIN 1

// ---------------- PWM outputs ----------------
#define OUT_COUNT 4
static const uint OUT_PINS[OUT_COUNT] = {6, 7, 8, 9};

// Output at 50 Hz RC PWM (20 ms frame)
#define OUT_PWM_HZ 50

// PWM clock at 1 tick = 1 us (easy mapping).
// Set PWM wrap to 20000 for 20 ms at 1 MHz tick rate.
#define PWM_TICK_HZ 1000000u
#define PWM_WRAP_US (1000000u / OUT_PWM_HZ) // 20000 us at 50 Hz

// ---------------- iBUS state ----------------
static uint8_t ibus_buf[IBUS_FRAME_LEN];
static int ibus_pos = 0;

static uint16_t ch_raw[IBUS_CH_COUNT]; // raw 16-bit channel values from frame
static uint32_t last_good_frame_us = 0;

// ---------------- Utility ----------------
static inline uint32_t now_us(void) {
  return to_us_since_boot(get_absolute_time());
}

static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

// iBUS checksum: 0xFFFF - sum(bytes[0..29]) == uint16(frame[30..31],
// little-endian)
static bool ibus_checksum_ok(const uint8_t *f) {
  uint16_t sum = 0;
  for (int i = 0; i < 30; i++)
    sum += f[i];
  uint16_t chk = (uint16_t)f[30] | ((uint16_t)f[31] << 8);
  return (uint16_t)(0xFFFF - sum) == chk;
}

// Poll UART, return true when a *new valid* frame was decoded
static bool ibus_poll_decode(void) {
  while (uart_is_readable(IBUS_UART)) {
    uint8_t b = uart_getc(IBUS_UART);

    // Better resync: if we're mid-frame and see header0, restart at pos=1
    if (ibus_pos > 0 && b == IBUS_HEADER0) {
      ibus_buf[0] = b;
      ibus_pos = 1;
      continue;
    }

    if (ibus_pos == 0) {
      if (b != IBUS_HEADER0)
        continue;
    } else if (ibus_pos == 1) {
      if (b != IBUS_HEADER1) {
        ibus_pos = 0;
        continue;
      }
    }

    ibus_buf[ibus_pos++] = b;

    if (ibus_pos == IBUS_FRAME_LEN) {
      ibus_pos = 0;

      if (!ibus_checksum_ok(ibus_buf)) {
        return false; // bad frame; keep trying
      }

      // Extract channels (little endian)
      for (int ch = 0; ch < IBUS_CH_COUNT; ch++) {
        int idx = 2 + ch * 2;
        ch_raw[ch] =
            (uint16_t)ibus_buf[idx] | ((uint16_t)ibus_buf[idx + 1] << 8);
      }

      last_good_frame_us = now_us();
      return true;
    }
  }
  return false;
}

// ---------------- PWM output setup ----------------
static void pwm_init_slice_if_needed(uint slice, bool *slice_inited) {
  if (slice_inited[slice])
    return;

  // Set PWM clock divider to make the PWM counter tick at 1 MHz.
  uint32_t sys_hz = clock_get_hz(clk_sys);
  float div = (float)sys_hz / (float)PWM_TICK_HZ;
  pwm_set_clkdiv(slice, div);

  // Set period (wrap) to 20,000 us for 50 Hz
  pwm_set_wrap(slice, PWM_WRAP_US);

  pwm_set_enabled(slice, true);
  slice_inited[slice] = true;
}

static void pwm_out_init_all(void) {
  // Pico has a small number of PWM slices; track which ones we configured.
  // Safe to size for 16 slices.
  bool slice_inited[16] = {0};

  for (int i = 0; i < OUT_COUNT; i++) {
    uint gpio = OUT_PINS[i];
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_init_slice_if_needed(slice, slice_inited);

    // Start with a safe value (1000 us)
    pwm_set_gpio_level(gpio, 1000);
  }
}

static inline void pwm_out_write_us(uint gpio, uint16_t pulse_us) {
  pwm_set_gpio_level(gpio, pulse_us);
}

// ---------------- arming + mixing ----------------
static bool armed = false;
static uint32_t arm_start_us = 0;

static bool failsafe_active(void) {
  // If we have never received a frame, treat as failsafe
  if (last_good_frame_us == 0)
    return true;
  return (now_us() - last_good_frame_us) > FAILSAFE_TIMEOUT_US;
}

static void update_arming_logic(uint16_t thr, uint16_t yaw) {
  const uint16_t THR_LOW = 1050;
  const uint16_t YAW_ARM = 1900;

  if (!armed) {
    if (thr < THR_LOW && yaw > YAW_ARM) {
      if (arm_start_us == 0)
        arm_start_us = now_us();
      if (now_us() - arm_start_us > 500000)
        armed = true;
    } else {
      arm_start_us = 0;
    }
  } else {
    const uint16_t YAW_DISARM = 1100;
    if (thr < THR_LOW && yaw < YAW_DISARM) {
      if (arm_start_us == 0)
        arm_start_us = now_us();
      if (now_us() - arm_start_us > 500000)
        armed = false;
    } else {
      arm_start_us = 0;
    }
  }
}

static void compute_and_write_outputs(void) {
  if (failsafe_active() || !armed) {
    for (int i = 0; i < OUT_COUNT; i++) {
      pwm_out_write_us(OUT_PINS[i], 1000);
    }
    return;
  }

  // Use CH3 as throttle (common RC mapping)
  uint16_t thr = clamp_u16(ch_raw[2], CH_MIN_US, CH_MAX_US);

  // Placeholder: throttle to all outputs
  for (int i = 0; i < OUT_COUNT; i++) {
    pwm_out_write_us(OUT_PINS[i], thr);
  }
}

int main() {
  stdio_init_all();
  sleep_ms(1200);

  // UART RX-only setup for iBUS
  uart_init(IBUS_UART, IBUS_BAUD);
  gpio_set_function(IBUS_UART_RX_PIN, GPIO_FUNC_UART);
  uart_set_format(IBUS_UART, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(IBUS_UART, true);

  // PWM outputs
  pwm_out_init_all();

  uint32_t last_print = now_us();

  const uint LED_PIN = PICO_DEFAULT_LED_PIN;

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  int i = 0;

  // int MAX_LINE_LENGTH = 255;
  // char input_line[MAX_LINE_LENGTH];
  while (true) {
    // int c = getchar();
    //
    // if (c != PICO_ERROR_TIMEOUT && c != EOF) {
    //   putchar(c);
    //
    //   if (c == '\n' || c == '\r') {
    //     input_line[i] = '\0';
    //
    //     if (i > 0) {
    //       printf("\nReceived line: %s\n", input_line);
    //
    //       i = 0;
    //     }
    //   } else {
    //     if (i < MAX_LINE_LENGTH) {
    //       input_line[i++] = (char)c;
    //     }
    //   }
    // }

    // printf("Toggling LED on \n");
    // gpio_put(LED_PIN, 1);
    // sleep_ms(1000);
    //
    // printf("Toggling LED off \n");
    // gpio_put(LED_PIN, 0);
    // sleep_ms(1000);

    (void)ibus_poll_decode();

    if (!failsafe_active()) {
      uint16_t thr = clamp_u16(ch_raw[2], CH_MIN_US, CH_MAX_US);
      uint16_t yaw = clamp_u16(ch_raw[3], CH_MIN_US, CH_MAX_US);
      update_arming_logic(thr, yaw);
    } else {
      armed = false;
    }

    compute_and_write_outputs();

    if (now_us() - last_print > 100000) {
      last_print = now_us();
      printf("FS=%d armed=%d CH1-4: %u %u %u %u\n", failsafe_active(), armed,
             ch_raw[0], ch_raw[1], ch_raw[2], ch_raw[3]);
    }

    tight_loop_contents();
  }
}
