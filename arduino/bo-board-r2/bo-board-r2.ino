 #include "decay.h"
#include "conf.h"
#include "pt.h"

#include "Adafruit_MPL115A2.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_DRV2605.h"

//#define SYNAPSE
//#define CAMPFIRE
#define SPEAKERS

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
  static uint32_t   last_sync = 0;
  static uint8_t    sync_round = 0;

  static uint8_t    am_leader = 0;
  static uint8_t    last_track_seen = 0;
  static uint32_t   last_round_time = 0;

  static uint8_t    my_volume = 0;

  static uint8_t    host_buf[256];
  static uint8_t    host_buf_len = 0;
  
  static uint8_t    buf[3];
  static int        rcv = 0;
  static uint8_t    next_is_address = 0;
  
  PT_BEGIN(pt);

  // CH2 (A9) is an analog input in this configuration!
  pinMode(A9, INPUT);

  // Right IO is driven, start high
  pinMode(RIGHT_IO_PIN, OUTPUT);
  digitalWrite(RIGHT_IO_PIN, HIGH);

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

  // If left side stays low for 50ms, you are leader
  if (digitalRead(LEFT_IO_PIN) == LOW) {
    last = millis();
    PT_WAIT_UNTIL(pt, millis() > (last + 50));
    if (digitalRead(LEFT_IO_PIN) == LOW) {
      am_leader = 1;
    }
  }

  // Give everyone time to boot up
  last = millis();
  PT_WAIT_UNTIL(pt, millis() > (last + 5000));

  while (1) {

    // Small yield
    PT_YIELD(pt);

    // Kludgy overrun prevention on headless systems
    if (host_buf_len > 200) host_buf_len = 0;

    // Always copy bus messages to Pi, making note of last track seen
    rcv = Serial1.read();
    if (rcv >= 0) {
      host_buf[host_buf_len] = rcv;
      host_buf_len++;
      if (rcv == 225) {
        digitalWrite(LOAD1_CTRL_PIN, !digitalRead(LOAD1_CTRL_PIN));
      }
      if (next_is_address) {
        last_track_seen = rcv;
        next_is_address = 0;
      }
      if (rcv == 0x0A) {
        next_is_address = 1;
      }
    }

    // Serial write conditions is either
    // - Am leader and it has been 200ms since last start, or
    // - Left IO is low (my turn)

    if ( ( am_leader && (millis() > (last_round_time + 200))) ||
         (!am_leader && (digitalRead(LEFT_IO_PIN) == LOW))) {

      last_round_time = millis();

      // Calculate my volume
      if (analogRead(A9) > 512) {
        my_volume = 255;
      } else {
        my_volume = 0;
      }

      // Build volume update packet
      if (am_leader) {
        buf[0] = 0;
      } else {
        buf[0] = last_track_seen + 1;
      }
      buf[1] = my_volume;
      buf[2] = '\n';

      // Is it time to sync (re-start tracks)?
      // Current track is 1:50 (110 seconds, round up to 115 for 5% error)
      if (am_leader && (millis() > (last_sync + 115000))) {
        last_sync = millis();
        buf[0] = 0xCC;
        sync_round = 1;
      } else {
        sync_round = 0;
      }

      // Write to Pi
      host_buf[host_buf_len] = buf[0]; host_buf_len++;
      host_buf[host_buf_len] = buf[1]; host_buf_len++;
      host_buf[host_buf_len] = buf[2]; host_buf_len++;

      // Write to bus
      bus_tx(buf, 3);

      // Wait 30ms, dumping updates to host
      last = millis();
      Serial.write(host_buf, host_buf_len);
      PT_WAIT_UNTIL(pt, millis() > (last + 30));
      host_buf_len = 0;

      // Signal next participant for 5ms, unless syncing
      if (sync_round == 0) {
        digitalWrite(RIGHT_IO_PIN, LOW);
        last = millis();
        PT_WAIT_UNTIL(pt, millis() > (last + 5));
        digitalWrite(RIGHT_IO_PIN, HIGH);
      }
    }
  }
  PT_END(pt);
}
