// --- Byteshare debug
// #define BYTESHARE_DEBUG

#define SOF_BYTE    0xAA

uint8_t out_buffer[3];

uint8_t in_addr     = 0;
uint8_t in_len      = 0;

uint8_t my_byte     = 0;
uint8_t shared_bytes[16];

uint32_t  bsp       = 9999999;
uint8_t   bnm       = 16;

uint8_t am_leader   = 0;
uint8_t addr_last   = 0;
uint8_t addr        = 0;

uint8_t byteshare_initialized = 0;

void byteshare_init(uint32_t ms_period, uint8_t n_members) {

  uint32_t last = 0;

  // Assume bus layer is initialized separately

  // Copy config vars
  bsp = ms_period;
  bnm = n_members;
  for (uint8_t i=0; i<n_members; i++) {
    shared_bytes[i] = 0;
  }

  // Right IO is driven, start high
  pinMode(RIGHT_IO_PIN, OUTPUT);
  digitalWrite(RIGHT_IO_PIN, HIGH);

  // Give everyone time to boot up
  delay(5000);

  // If left side stays low for 50ms, you are leader
  if (digitalRead(LEFT_IO_PIN) == LOW) {
    last = millis();
    delay(50);
    if (digitalRead(LEFT_IO_PIN) == LOW) {
      am_leader = 1;
    }
  }

  byteshare_initialized = 1;

}

uint8_t* byteshare_array(void) {
  return shared_bytes;
}

uint8_t byteshare_get(uint8_t addr) {
  return shared_bytes[addr];
}

void byteshare_set(uint8_t shared_byte) {
  my_byte = shared_byte;
  shared_bytes[addr] = my_byte;
}

void byteshare_thread(struct pt *pt) {

  static uint32_t last = 0;
  static uint32_t last_round_time = 0;
  static int res;

  PT_BEGIN(pt);

  while (1) {

  if (!bus_initialized) {
    PT_YIELD(pt);
    continue;
  }

  // Send our byte if either
  //     We are leader and it is update time
  //     We have received token (signal low) on left side
  if ( ( am_leader && (millis() > (last_round_time + bsp))) ||
       (!am_leader && (digitalRead(LEFT_IO_PIN) == LOW))) { 

    #ifdef BYTESHARE_DEBUG
    Serial.println("Sending our byteshare packet");
    Serial.print("Currently ");
    Serial.print(shared_bytes[0]); Serial.print(", ");
    Serial.print(shared_bytes[1]); Serial.print(", ");
    Serial.println(shared_bytes[2]);
    #endif

    last_round_time = millis();

    if (!am_leader) {
      addr = addr_last + 1;
    }

    out_buffer[0] = SOF_BYTE;
    out_buffer[1] = addr;
    out_buffer[2] = my_byte;

    // Send and wait 30ms
    bus_tx(out_buffer, 3);
    last = millis();
    PT_WAIT_UNTIL(pt, millis() > (last + 30));

    // Signal next participant for 5ms
    digitalWrite(RIGHT_IO_PIN, LOW);
    last = millis();
    PT_WAIT_UNTIL(pt, millis() > (last + 5));
    digitalWrite(RIGHT_IO_PIN, HIGH);

  }

  // Do we have incoming bytes to sort through?
  res = Serial1.read();
  if (res >= 0) {
    // Serial.print("Got byte "); Serial.println(res);

    if (in_len == 0 && res == SOF_BYTE) {
      in_len = 1;
    }

    else if (in_len == 1) {
      in_addr = res;
      if (res < bnm)  in_len = 2;
      else            in_len = 0;
    }

    else if (in_len == 2) {
      shared_bytes[in_addr] = res;
      addr_last = in_addr;
      in_len = 0;
      #ifdef BYTESHARE_DEBUG
      Serial.print("Got "); Serial.print(res); Serial.print(" from "); Serial.println(in_addr);
      #endif
    }
  }

  // Yield for other threads
  PT_YIELD(pt);

  }

PT_END(pt);

}
