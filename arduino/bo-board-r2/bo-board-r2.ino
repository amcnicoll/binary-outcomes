#include "decay.h"
#include "conf.h"
#include "pt.h"

#include "Adafruit_MPL115A2.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_DRV2605.h"

//#define SYNAPSE
#define CAMPFIRE
//#define SPEAKERS

state_t state;

static struct pt app_thread_pt, bus_thread_pt;

void setup() {

  // Initialize serial
  Serial.begin(9600);     // USB

  // Pin direction initialize
  pinMode(LOAD1_CTRL_PIN, OUTPUT);
  pinMode(LOAD2_CTRL_PIN, OUTPUT);

  // Initial drive
  analogWrite(LOAD1_CTRL_PIN, 0);
  analogWrite(LOAD2_CTRL_PIN, 0);

  // Initialize pairwise to inputs, pulled up
  pinMode(LEFT_IO_PIN,  INPUT_PULLUP);
  pinMode(RIGHT_IO_PIN, INPUT_PULLUP);

  // Initialize state
  state.address           = 0;
  state.pressure          = 0;
  state.activated         = 0;
  state.ch_a_target       = 0;
  state.ch_a_current      = 0;
  state.ch_b_target       = 0;
  state.ch_b_current      = 0;
  state.synapse_on_time   = 0;
  state.synapse_off_time  = 0;
  state.synapse_leftward  = 0;
  state.synapse_rightward = 0;

  // Initialize threads
  PT_INIT(&app_thread_pt);
  PT_INIT(&bus_thread_pt);
}

void loop() {

  // Service app thread
  #ifdef SYNAPSE
  app_synapse(&app_thread_pt);
  #endif
  #ifdef CAMPFIRE
  app_campfire(&app_thread_pt);
  #endif
  #ifdef SPEAKERS
  app_speakers(&app_thread_pt);
  #endif

  // Service bus thread
  #ifdef SPEAKERS
  bus_thread(&bus_thread_pt);
  #endif
}

void app_synapse(struct pt *pt) {

  static Adafruit_BMP280   sensor;
  static float      avg_pressure;

  static uint32_t   last = 0;
  static bool       pressured;
  static float      this_pressure;

  PT_BEGIN(pt);

  // Initiailize pressure sensor
  sensor.begin();

  // Default settings from datasheet
  sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  avg_pressure = sensor.readPressure();
  
  while (1) {

    // Get new pressure, check if pressured
    this_pressure = sensor.readPressure();
    pressured = this_pressure > (SYNAPSE_PRESSURE_DIFF + avg_pressure);

    // Update pressure average
    avg_pressure *= 0.98;
    avg_pressure += this_pressure * 0.02;

    // Check for activation conditions
    if (!state.activated) {
      
      // If we have a pressure activation, generate synapse on both sides
      if (pressured) {
        state.activated = 1;
        state.synapse_leftward = 1;
        state.synapse_rightward = 1;
      }

      // If we have left synapse, propagate to right
      if (digitalRead(LEFT_IO_PIN) == LOW) {
        state.activated = 1;
        state.synapse_rightward = 1;
      }

      // If we have right synapse, propagate to left
      if (digitalRead(RIGHT_IO_PIN) == LOW) {
        state.activated = 1;
        state.synapse_leftward = 1;
      }

      // If we are newly activated, set LED target high
      if (state.activated) {
        state.ch_a_target = UINT16_MAX;
        state.synapse_on_time = millis() + SYNAPSE_LATENCY_MS;
        state.synapse_off_time = state.synapse_on_time + SYNAPSE_LENGTH_MS;
      }
    }
  
    // Update fade up
    if (state.ch_a_target > state.ch_a_current) {
      if (UINT16_MAX - state.ch_a_current < LED_FADE_RATE) {
        state.ch_a_current = UINT16_MAX;
        state.ch_a_target  = 0; // Go back down
      } else {
        state.ch_a_current += LED_FADE_RATE;
      }

    // Update fade down
    } else if (state.ch_a_target < state.ch_a_current) {
      if (state.ch_a_current < LED_FADE_RATE) {
        state.ch_a_current = 0;
      } else {
        state.ch_a_current -= LED_FADE_RATE;
      }
    }

    // Update synapse signals
    if (millis() > state.synapse_off_time) {
      pinMode(LEFT_IO_PIN,  INPUT_PULLUP);
      pinMode(RIGHT_IO_PIN, INPUT_PULLUP);
    }

    else if (millis() > state.synapse_on_time) {
      if (state.synapse_leftward && digitalRead(LEFT_IO_PIN)) {
        pinMode(LEFT_IO_PIN, OUTPUT);
        digitalWrite(LEFT_IO_PIN, LOW);
      }
      if (state.synapse_rightward && digitalRead(RIGHT_IO_PIN)) {
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

    // Actualize current PWM channels
    analogWrite(LOAD1_CTRL_PIN, duty_lookup(state.ch_a_current));
    analogWrite(LOAD2_CTRL_PIN, duty_lookup(state.ch_b_current));

    // Wait 10ms
    last = millis();
    PT_WAIT_UNTIL(pt, millis() > (last + 10));

  }
  PT_END(pt);
}

void app_campfire(struct pt *pt) {

  static Adafruit_MPL115A2 sensor;
  static float      avg_pressure;

  static uint32_t   last = 0;
  static bool       butt;
  static float      this_pressure;
  static float      delta;
  static uint8_t    bus_count = 0;

  PT_BEGIN(pt);

  // Initiailize pressure sensor
  sensor = Adafruit_MPL115A2();
  sensor.begin();
  
  avg_pressure = sensor.getPressure();

  // Initialize bus to output, driven down
  // This is because "all released" is the global sit signal
  pinMode(BUS_IO_PIN, OUTPUT);
  digitalWrite(BUS_IO_PIN, LOW);

  // No butt
  butt = false;
  digitalWrite(LOAD1_CTRL_PIN, HIGH);
  digitalWrite(LOAD2_CTRL_PIN, HIGH);
  
  while (1) {

    // Get new pressure, calculate delta from running average
    this_pressure = sensor.getPressure();
    delta         = this_pressure - avg_pressure;

    // Update pressure average
    avg_pressure *= 0.98;
    avg_pressure += this_pressure * 0.02;

    // If we are passing from non-butt to butt, enable channel A, release bus
    if ((!butt) && (delta > CAMPFIRE_PRESSURE_DIFF)) {
      digitalWrite(LOAD1_CTRL_PIN, LOW);
      pinMode(BUS_IO_PIN, INPUT_PULLUP);
      butt = true;
      //Serial.println("Butt!");
    }

    // If we are passing from butt to non-butt, drive bus low and don't enable channel A
    if ((butt) && (delta < (-CAMPFIRE_PRESSURE_DIFF))) {
      digitalWrite(LOAD1_CTRL_PIN, HIGH);
      pinMode(BUS_IO_PIN, OUTPUT);
      digitalWrite(BUS_IO_PIN, LOW);
      butt = false;
      //Serial.println("Unbutt!");
    }

    #define GLOBAL_LIGHT_DELAY  1

    // If the bus is high for 100 ticks (1s), drive channel B
    if (digitalRead(BUS_IO_PIN)){
      if (bus_count < GLOBAL_LIGHT_DELAY) {
        bus_count++;
      }
    } else if (bus_count > 0) {
      bus_count--;
    }
    
    if (bus_count == GLOBAL_LIGHT_DELAY) {
      digitalWrite(LOAD2_CTRL_PIN, LOW);
    } else {
      digitalWrite(LOAD2_CTRL_PIN, HIGH);
    }

    // Wait 10ms
    last = millis();
    PT_WAIT_UNTIL(pt, millis() > (last + 10));

  }
  PT_END(pt);
}

void app_speakers(struct pt *pt) {

  static Adafruit_DRV2605 drv;

  static uint32_t   last = 0;
  static uint32_t   last_keepalive = 0;

  static uint8_t    my_track = 2;
  static uint8_t    my_volume = 0;
  static uint8_t    last_volume = 0;

  static uint8_t    n_tracks = 5;
  static uint8_t    volumes[4] = {255, 150, 50, 0};

  static uint8_t    buf[3];

  static uint8_t    t = 0;
  static uint8_t    v = 0;
  
  PT_BEGIN(pt);

  // CH2 (A9) is an analog input in this configuration!
  pinMode(A9, INPUT);

  // Set buzzer driver in audio input mode
  drv.begin();
  drv.setMode(DRV2605_MODE_AUDIOVIBE);              // Audio input
  drv.writeRegister8(DRV2605_REG_CONTROL1, 0x20);   // 
  drv.writeRegister8(DRV2605_REG_CONTROL3, 0xA3);

  // Full scale (255) is 1.8 Vpp for audio
  // We only have about 100mV Vpp (really up to 200mV with volume knob)
  // So divide by 18 - set the ATH_MAX_INPUT to 14.
  // Also, lower the deadband - DRV2605_REG_AUDIOLVL to 3.
  drv.writeRegister8(DRV2605_REG_AUDIOLVL, 3);
  drv.writeRegister8(DRV2605_REG_AUDIOMAX, 14);

  // Test buzz to confirm driver and motor function
  // drv.setMode(DRV2605_MODE_REALTIME);
  // drv.setRealtimeValue(50 );

  while (1) {

    // Calculate my volume
    if (analogRead(A9) > 512) {
      my_volume = 255;
    } else {
      my_volume = 0;
    }

    // Update volume change to serial if needed
    // OR if it has been more than 3 seconds (keepalive)
    if ( (my_volume != last_volume) || (millis() > (last_keepalive + 3000)) ) {

      buf[0] = my_track;
      buf[1] = my_volume;
      buf[2] = '\n';
      Serial.write(buf, 3);
      last_keepalive = millis();

      // Rate limit changes
      last = millis();
      PT_WAIT_UNTIL(pt, millis() > (last + 250));
    }

    last_volume = my_volume;

    // Fast poll rate
    last = millis();
    PT_WAIT_UNTIL(pt, millis() > (last + 10));

  }
  PT_END(pt);
}
