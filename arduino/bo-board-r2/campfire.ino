#include "Adafruit_MPL115A2.h"

// --- Campfire Parameters
#define CAMPFIRE_PRESSURE_DIFF      0.35

// --- Compiler switch for plotting pressure
// #define CAMPFIRE_PLOT_PRESSURE

void app_campfire(struct pt *pt) {

  static Adafruit_MPL115A2 sensor;
  static float      avg_pressure;

  static uint32_t   last = 0;
  static float      this_pressure;
  static float      this_pressure_smooth;
  static float      delta;

  static uint8_t    my_butt   = 0;
  static uint8_t*   all_butts;

  PT_BEGIN(pt);

  // Initialize bus
  bus_init();

  // Initialize byteshare protocol
  byteshare_init(200, 3);

  // Initialize pressure sensor
  sensor = Adafruit_MPL115A2();
  sensor.begin();

  avg_pressure = sensor.getPressure();

  while (1) {

    // Make lights match butt status

    // Global: either no butts or all butts
    all_butts = byteshare_array();
    if ( (all_butts[0] + all_butts[1] + all_butts[2] == 0) ||
         (all_butts[0] + all_butts[1] + all_butts[2] == 3) ) {
      digitalWrite(LOAD2_CTRL_PIN, LOW);
    } else {
      digitalWrite(LOAD2_CTRL_PIN, HIGH);
    }

    // Local: Always follow our butt
    if (my_butt) {
      digitalWrite(LOAD1_CTRL_PIN, LOW);
    } else {
      digitalWrite(LOAD1_CTRL_PIN, HIGH);
    }

    // Get new pressure, calculate delta from running average
    this_pressure = sensor.getPressure();

    // Smooth this pressure
    this_pressure_smooth *= 0.75;
    this_pressure_smooth += this_pressure * 0.25;

    // Update pressure average
    avg_pressure *= 0.98;
    avg_pressure += this_pressure * 0.02;

    // Update our butt status
    delta = this_pressure_smooth - avg_pressure;
    if ((!my_butt) && (delta > CAMPFIRE_PRESSURE_DIFF)) {
      my_butt = 1;
    }
    if ((my_butt) && (delta < (-CAMPFIRE_PRESSURE_DIFF))) {
      my_butt = 0;
    }
    byteshare_set(my_butt);

    // Debug
    #ifdef CAMPFIRE_PLOT_PRESSURE
    //Serial.print(this_pressure_smooth);
    //Serial.print(", ");
    Serial.println(10*delta);
    #endif

    // Wait 10ms
    last = millis();
    PT_WAIT_UNTIL(pt, millis() > (last + 10));

  }
  PT_END(pt);
}
