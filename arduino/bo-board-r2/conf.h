#include <stdint.h>

#ifndef CONF_H
#define CONF_H

#define RX_PIN          0
#define TX_PIN          1
#define DE_PIN          15
#define NRE_PIN         14
#define LEFT_IO_PIN     A3
#define RIGHT_IO_PIN    4
#define BUS_IO_PIN      5
#define LOAD1_CTRL_PIN  10
#define LOAD2_CTRL_PIN  9
#define BUS_IO_PIN_NC   A2

#define LED_FADE_RATE   800

#define BUS_CLAIM_MS    5
#define BUS_CLEAN_MS    2

#define SYNAPSE_LATENCY_MS          500
#define SYNAPSE_LENGTH_MS           100
#define SYNAPSE_PRESSURE_DIFF       50
#define SYNAPSE_MUTE_DURATION       5000

#define CAMPFIRE_PRESSURE_DIFF      0.35
#define GLOBAL_LIGHT_DELAY          1

typedef struct __attribute__((packed)) {
  uint8_t       address;
  uint8_t       activated;
  uint16_t      pressure;
  uint16_t      ch_a_target;
  uint16_t      ch_a_current;
  uint16_t      ch_b_target;
  uint16_t      ch_b_current;
  uint32_t      synapse_on_time;
  uint32_t      synapse_off_time;
  uint8_t       synapse_leftward;
  uint8_t       synapse_rightward;
  uint32_t      mute_end;
} state_t;

#endif
