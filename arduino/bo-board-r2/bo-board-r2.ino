#include "pt.h"
#include <stdint.h>

// --- Application selection
//#define IMPULSE
//#define CAMPFIRE
#define OUTCOMES

// --- Pins
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

static struct pt  app_thread_pt,
                  bus_thread_pt,
                  byteshare_thread_pt;

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

  // Initialize threads
  PT_INIT(&app_thread_pt);
  PT_INIT(&bus_thread_pt);
  PT_INIT(&byteshare_thread_pt);
}

void loop() {

  #ifdef IMPULSE
    app_impulse(&app_thread_pt);
  #endif
  #ifdef CAMPFIRE
    app_campfire(&app_thread_pt);
    bus_thread(&bus_thread_pt);
    byteshare_thread(&byteshare_thread_pt);
  #endif
  #ifdef OUTCOMES
    app_outcomes(&app_thread_pt);
    bus_thread(&bus_thread_pt);
  #endif

}
