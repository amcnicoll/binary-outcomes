#include "Adafruit_DRV2605.h"

void app_outcomes(struct pt *pt) {

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

  // Initialize bus
  bus_init();

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
