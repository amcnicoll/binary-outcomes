#include "Adafruit_BMP280.h"
#include "decay.h"

// --- Impulse Parameters
#define IMPULSE_LATENCY_MS          500
#define IMPULSE_LENGTH_MS           100
#define IMPULSE_PRESSURE_DIFF       50
#define IMPULSE_MUTE_DURATION       5000
#define IMPULSE_FADE_RATE           800

// --- Compiler switch for flicker units
#define IMPULSE_FLICKER

// --- Compiler switch for debug plotting
// #define IMPULSE_PLOT_PRESSURE

typedef struct __attribute__((packed)) {
  uint8_t       address;
  uint8_t       activated;
  uint16_t      pressure;
  uint16_t      ch_a_target;
  uint16_t      ch_a_current;
  uint16_t      ch_b_target;
  uint16_t      ch_b_current;
  uint32_t      impulse_on_time;
  uint32_t      impulse_off_time;
  uint8_t       impulse_leftward;
  uint8_t       impulse_rightward;
  uint32_t      mute_end;
} state_t;

void app_impulse(struct pt *pt) {

  static state_t    state;

  static Adafruit_BMP280   sensor;
  static float      avg_pressure;

  static uint32_t   last = 0;
  static bool       pressured;
  static float      this_pressure;

  static uint8_t flicker_table[20] = {255, 255, 255, 255, 255,
                                      255, 255, 255, 255, 255,
                                      200, 0,   0,   200, 200,
                                      0,   0,   0,   0,   255
                                     };
  static uint32_t flicker_table_idx;

  static uint16_t duty_to_write;

  PT_BEGIN(pt);

  // Initialize state
  state.address           = 0;
  state.pressure          = 0;
  state.activated         = 0;
  state.ch_a_target       = 0;
  state.ch_a_current      = 0;
  state.ch_b_target       = 0;
  state.ch_b_current      = 0;
  state.impulse_on_time   = 0;
  state.impulse_off_time  = 0;
  state.impulse_leftward  = 0;
  state.impulse_rightward = 0;
  state.mute_end          = 0;

  // Initialize pressure sensor
  sensor.begin();

  // Default settings from datasheet
  sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  avg_pressure = sensor.readPressure();

  while (1) {

    // If we are past mute end, bus signal is released with pullup
    if (millis() >= state.mute_end) {
      pinMode(BUS_IO_PIN, INPUT_PULLUP);
    }

    // Get new pressure, check if pressured
    this_pressure = sensor.readPressure();
    pressured = this_pressure > (IMPULSE_PRESSURE_DIFF + avg_pressure);

    // Update pressure average
    avg_pressure *= 0.98;
    avg_pressure += this_pressure * 0.02;

    // Debug
    #ifdef IMPULSE_PLOT_PRESSURE
    Serial.print(this_pressure);
    Serial.print(", ");
    Serial.print(avg_pressure);
    Serial.print(", ");
    if (state.activated) {
      Serial.println(98070);
    } else {
      Serial.println(98050);
    }
    #endif

    // Check for activation conditions
    if (!state.activated) {

      // If we have a pressure activation and there is no mute active,
      // generate impulse on both sides and start mute
      if (pressured && (digitalRead(BUS_IO_PIN) != LOW)) {
        state.activated = 1;
        state.impulse_leftward = 1;
        state.impulse_rightward = 1;
        pinMode(BUS_IO_PIN, OUTPUT);
        digitalWrite(BUS_IO_PIN, LOW);
        state.mute_end = millis() + IMPULSE_MUTE_DURATION;
      }

      // If we have left impulse, propagate to right
      if (digitalRead(LEFT_IO_PIN) == LOW) {
        state.activated = 1;
        state.impulse_rightward = 1;
      }

      // If we have right impulse, propagate to left
      if (digitalRead(RIGHT_IO_PIN) == LOW) {
        state.activated = 1;
        state.impulse_leftward = 1;
      }

      // If we are newly activated, set LED target high
      if (state.activated) {
        state.ch_a_target = UINT16_MAX;
        state.impulse_on_time = millis() + IMPULSE_LATENCY_MS;
        state.impulse_off_time = state.impulse_on_time + IMPULSE_LENGTH_MS;
        flicker_table_idx = 0;
      }
    }

    // Update fade up
    if (state.ch_a_target > state.ch_a_current) {
      if (UINT16_MAX - state.ch_a_current < IMPULSE_FADE_RATE) {
        state.ch_a_current = UINT16_MAX;
        state.ch_a_target  = 0; // Go back down
      } else {
        state.ch_a_current += IMPULSE_FADE_RATE;
      }

      // Update fade down
    } else if (state.ch_a_target < state.ch_a_current) {
      if (state.ch_a_current < IMPULSE_FADE_RATE) {
        state.ch_a_current = 0;
      } else {
        state.ch_a_current -= IMPULSE_FADE_RATE;
      }
    }

    // Update impulse signals
    if (millis() > state.impulse_off_time) {
      pinMode(LEFT_IO_PIN,  INPUT_PULLUP);
      pinMode(RIGHT_IO_PIN, INPUT_PULLUP);
    }

    else if (millis() > state.impulse_on_time) {
      if (state.impulse_leftward && digitalRead(LEFT_IO_PIN)) {
        pinMode(LEFT_IO_PIN, OUTPUT);
        digitalWrite(LEFT_IO_PIN, LOW);
      }
      if (state.impulse_rightward && digitalRead(RIGHT_IO_PIN)) {
        pinMode(RIGHT_IO_PIN, OUTPUT);
        digitalWrite(RIGHT_IO_PIN, LOW);
      }
    }

    // If we're activated and current is at 0, we're done
    if (state.activated && state.ch_a_current == 0) {
      state.activated = 0;
    }

    // Mirror channels
    state.ch_b_current = state.ch_a_current;

    // Calculate PWM to set
    duty_to_write = duty_lookup(state.ch_a_current);

    // Superimpose flicker
    #ifdef IMPULSE_FLICKER
    if (millis() > state.impulse_on_time && flicker_table_idx < sizeof(flicker_table)) {
      flicker_table_idx = (millis() - state.impulse_on_time) >> 6;
      duty_to_write = (duty_to_write * flicker_table[flicker_table_idx]) >> 8;
    }
    #endif

    // Write PWM
    analogWrite(LOAD1_CTRL_PIN, duty_to_write);
    analogWrite(LOAD2_CTRL_PIN, duty_to_write);

    // Wait 10ms
    last = millis();
    PT_WAIT_UNTIL(pt, millis() > (last + 10));

  }
  PT_END(pt);
}
