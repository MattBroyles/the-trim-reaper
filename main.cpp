#include <cstdint>
#include <pico/stdio.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

// ============================================================
// iBUS definitions
// ============================================================
#define IBUS_BAUD 115200

// Fixed iBUS packet size, in bytes
#define IBUS_FRAME_LEN 32

// Two byte iBUS frame header
#define IBUS_HEADER0 0x20
#define IBUS_HEADER1 0x40

// Number of channels carried in one iBUS frame
#define IBUS_CH_COUNT 14

// Trigger a failsafe is no valid frame is received for this long
#define FAILSAFE_TIMEOUT_US 100000 // 100 ms

// Expected RC PWM-equivalent limits, in microseconds
#define CH_MIN_US 1000
#define CH_MAX_US 2000
#define CH_CENTER_US 1500

// UART + GPIO pin used for iBUS input
#define IBUS_UART uart0
#define IBUS_UART_RX_PIN 1

// ============================================================
// Sabertooth Packetized Serial
// ============================================================

// UART used to communicate with the Sabertooth motor controller
#define SABER_UART uart1
#define SABER_UART_TX_PIN 8

#define SABER_BAUD 9600
#define SABER_ADDR 128 // DIP switch address

// ============================================================
// Mixer tuning
// ============================================================

// Stick deadzone around center
static const float DEADZONE = 0.05f;

// Exponential response amount. 0 = linear, 1 = full cubic
static const float EXPO = 0.40f;

// Steering authority at rest vs full speed
static const float TURN_AT_ZERO_SPEED = 1.00f;
static const float TURN_AT_FULL_SPEED = 0.35f;

// Max change in output per second
static const float SLEW_PER_SEC = 2.5f;

// Absolute output clamp
static const float MAX_OUTPUT = 1.0f;

// Channel indices within iBUS packet
static const int CH_THROTTLE = 2;
static const int CH_STEER = 3;
static const int CH_ARM = 6;

// ============================================================
// State
// ============================================================

// Raw iBUS frame buffer and capture position
static uint8_t ibus_buf[IBUS_FRAME_LEN];
static uint8_t ibus_pos = 0;

// Decoded RC channel values, in microseconds
static uint16_t ch_raw[IBUS_CH_COUNT];

// Timestamp of last valid RC frame
static uint32_t last_good_frame_us = 0;

// Arming state and timer
static bool armed = false;
static uint32_t arm_start_us = 0;

// Motor command outputs (-1..1)
static float left_cmd = 0.0f;
static float right_cmd = 0.0f;

// Timestamp of last mixer update, used for slew-rate limiting
static uint32_t last_mix_us = 0;

// ============================================================
// Utility
// ============================================================

// Current time in microseconds since last boot
static inline uint32_t now_us(void) {
  return to_us_since_boot(get_absolute_time());
}

// Clamp unsigned 16-bit value into [low, high]
static inline uint16_t clamp_u16(uint16_t v, uint16_t low, uint16_t high) {
  if (v < low)
    return low;

  if (v > high)
    return high;

  return v;
}

// Clamp float into [low, high]
static inline float clamp_f(float v, float low, float high) {
  if (v < low)
    return low;

  if (v > high)
    return high;

  return v;
}

// Fast float absolute value
static inline float f_abs(float x) { return x < 0 ? -x : x; }

// ============================================================
// iBUS decode
// ============================================================

// Verify iBUS checksum:
// checksum = 0xFFFF - sum(bytes[0..29])
static bool ibus_checksum_ok(const uint8_t *f) {
  uint16_t sum = 0;

  for (int i = 0; i < 30; i++)
    sum += f[i];

  uint16_t chk = (uint16_t)f[30] | ((uint16_t)f[31] << 8);

  return (uint16_t)(0xFFFF - sum) == chk;
}

// Poll UART and attempt to assemble exactly one valid iBUS frame.
// Returns true when a good frame is decoded.
static bool ibus_poll_decode(void) {
  while (uart_is_readable(IBUS_UART)) {

    uint8_t b = uart_getc(IBUS_UART);

    // If we're in the middle of a frame and see header0, restart frame capture
    if (ibus_pos > 0 && b == IBUS_HEADER0) {
      ibus_buf[0] = b;
      ibus_pos = 1;
      continue;
    }

    // Sync the two header bytes
    if (ibus_pos == 0) {
      if (b != IBUS_HEADER0)
        continue;
    } else if (ibus_pos == 1) {
      if (b != IBUS_HEADER1) {
        ibus_pos = 0;
        continue;
      }
    }

    // Reset if buffer overruns
    if (ibus_pos >= IBUS_FRAME_LEN) {
      ibus_pos = 0;
      continue;
    }

    ibus_buf[ibus_pos++] = b;

    // Full frame received
    if (ibus_pos == IBUS_FRAME_LEN) {
      ibus_pos = 0;

      if (!ibus_checksum_ok(ibus_buf)) {
        return false;
      }

      // Extract 16-bit channel values, little-endian
      for (int ch = 0; ch < IBUS_CH_COUNT; ch++) {
        int idx = 2 + ch * 2;
        ch_raw[ch] =
            (uint16_t)ibus_buf[idx] | ((uint16_t)ibus_buf[idx + 1] << 8);
      }

      // Store timestamp for failsafe
      last_good_frame_us = now_us();
      return true;
    }
  }

  return false;
}

// ============================================================
// Sabertooth packet serial
// ============================================================

// Initialize UART for Sabertooth motor controller
static inline void saber_uart_init(void) {
  uart_init(SABER_UART, SABER_BAUD);
  gpio_set_function(SABER_UART_TX_PIN, GPIO_FUNC_UART);
  uart_set_format(SABER_UART, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(SABER_UART, true);
  sleep_ms(200);
}

// Send one Sabertooth packet: address, command, data, checksum
static inline void saber_send(uint8_t addr, uint8_t cmd, uint8_t data) {
  uint8_t chk = (addr + cmd + data) & 0x7F;
  uart_putc_raw(SABER_UART, addr);
  uart_putc_raw(SABER_UART, cmd);
  uart_putc_raw(SABER_UART, data);
  uart_putc_raw(SABER_UART, chk);
}

// Convert normalized [-1..1] to Sabertooth 7-bit speed [0..127]
static inline uint8_t saber_norm_to_7bit(float x) {

  x = clamp_f(x, -1.0f, 1.0f);

  int v = (int)(64.0f + x * 63.0f + (x >= 0 ? 0.5f : -0.5f));

  if (v < 0)
    v = 0;

  if (v > 127)
    v = 127;

  return (uint8_t)v;
}

// Send the left/right motor commands
static inline void saber_set(float left, float right) {
  saber_send(SABER_ADDR, 6, saber_norm_to_7bit(left));
  saber_send(SABER_ADDR, 7, saber_norm_to_7bit(right));
}

// ============================================================
// Mixer math
// ============================================================

// Convert RC PWM microseconds into normalized [-1..1]
static inline float us_to_norm(uint16_t us) {
  us = clamp_u16(us, CH_MIN_US, CH_MAX_US);

  if (us >= CH_CENTER_US)
    return (float)(us - CH_CENTER_US) / (float)(CH_MAX_US - CH_CENTER_US);

  else
    return (float)(us - CH_CENTER_US) / (float)(CH_CENTER_US - CH_MIN_US);
}

// Apply deadzone and rescale remaining range
static inline float apply_deadzone(float x) {
  float a = f_abs(x);

  if (a < DEADZONE)
    return 0.0f;

  float sign = (x >= 0) ? 1.0f : -1.0f;
  float mag = (a - DEADZONE) / (1.0f - DEADZONE);

  return sign * mag;
}

// Cubic exponential response curve
static inline float expo_curve(float x) {
  return (1.0f - EXPO) * x + EXPO * x * x * x;
}

// Limit slew: move forward by max delta
static inline float slew(float cur, float tgt, float maxd) {
  float d = tgt - cur;
  if (d > maxd)
    d = maxd;
  if (d < -maxd)
    d = -maxd;
  return cur + d;
}

// ============================================================
// Arming + output
// ============================================================

// True if RC signal lost or never received
static bool failsafe_active(void) {
  if (last_good_frame_us == 0)
    return true;
  return (now_us() - last_good_frame_us) > FAILSAFE_TIMEOUT_US;
}

static void update_arming(uint16_t thr, uint16_t yaw, uint16_t arm) {
  // Hardware arm switch must be high
  if (arm < 1950) {
    armed = false;
    arm_start_us = 0;
    return;
  }

  const uint16_t THR_LOW = 1050;
  const uint16_t YAW_ARM = 1900;

  if (!armed) {
    if (thr < THR_LOW && yaw > YAW_ARM) {
      if (!arm_start_us)
        arm_start_us = now_us();
      if (now_us() - arm_start_us > 500000)
        armed = true;
    } else
      arm_start_us = 0;
  } else {
    const uint16_t YAW_DIS = 1100;
    if (thr < THR_LOW && yaw < YAW_DIS) {
      if (!arm_start_us)
        arm_start_us = now_us();
      if (now_us() - arm_start_us > 500000)
        armed = false;
    } else
      arm_start_us = 0;
  }
}

// ============================================================
// Main mixer loop
// ============================================================

// Reads RC, mixes throttle + steering into differential outputs,
// applies slew limiting, and sends to the motor controller
static void update_motors(void) {
  // Safety cutoff
  if (failsafe_active() || !armed) {
    left_cmd = right_cmd = 0.0f;
    last_mix_us = now_us();
    saber_set(0.0f, 0.0f);
    return;
  }

  // Normalize RC inputs
  float throttle = us_to_norm(ch_raw[CH_THROTTLE]);
  float steering = us_to_norm(ch_raw[CH_STEER]);

  // Deadzone + expo shaping
  throttle = expo_curve(apply_deadzone(throttle));
  steering = expo_curve(apply_deadzone(steering));

  // Reduce steering authority at higher speeds
  float speed = f_abs(throttle);
  float turn_scale =
      TURN_AT_ZERO_SPEED + (TURN_AT_FULL_SPEED - TURN_AT_ZERO_SPEED) * speed;

  float turn = steering * turn_scale;

  // Differential mix
  float tgt_left = clamp_f(throttle + turn, -MAX_OUTPUT, MAX_OUTPUT);
  float tgt_right = clamp_f(throttle - turn, -MAX_OUTPUT, MAX_OUTPUT);

  // Time delta for slew limiter
  uint32_t t = now_us();
  float dt = (float)(t - last_mix_us) / 1000000.0f;
  last_mix_us = t;

  float maxd = SLEW_PER_SEC * dt;

  // Apply slew limiting
  left_cmd = slew(left_cmd, tgt_left, maxd);
  right_cmd = slew(right_cmd, tgt_right, maxd);

  // Send to motor controller
  saber_set(left_cmd, right_cmd);
}

// ============================================================
// main
// ============================================================
int main() {
  stdio_init_all();
  sleep_ms(1000);

  // Initialize iBUS UART
  uart_init(IBUS_UART, IBUS_BAUD);
  gpio_set_function(IBUS_UART_RX_PIN, GPIO_FUNC_UART);

  // Initialize Saebertooth UART
  saber_uart_init();

  while (true) {
    // Attempt to decode an RC frame
    ibus_poll_decode();

    // Update arming unless failsafe active
    if (!failsafe_active()) {
      update_arming(ch_raw[CH_THROTTLE], ch_raw[CH_STEER], ch_raw[CH_ARM]);
    } else {
      armed = false;
    }

    // Run mixer and output to controller
    update_motors();
    tight_loop_contents();
  }
}
