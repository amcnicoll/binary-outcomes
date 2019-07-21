
uint8_t* tx_msg_buffer;
uint8_t  tx_msg_len;

int      ard_buffer_size;

void bus_init(void) {

  // Initialize buffers
  tx_msg_len = 0;

  // Set pullup on receiver
  pinMode(RX_PIN, INPUT_PULLUP);

  // Initialize bus serial
  Serial1.begin(9600);
  ard_buffer_size = Serial1.availableForWrite();

  // Configure transceiver control pins
  pinMode(DE_PIN, OUTPUT);
  pinMode(NRE_PIN, OUTPUT);

  // Configure bus pin
  pinMode(BUS_IO_PIN, INPUT_PULLUP);

  // Release bus and set RX
  pinMode(BUS_IO_PIN, INPUT_PULLUP);
  bus_set_rx();
}

void bus_set_tx(void) {
  digitalWrite(DE_PIN, HIGH);
  digitalWrite(NRE_PIN, HIGH);
}

void bus_set_rx(void) {
  digitalWrite(DE_PIN, LOW);
  digitalWrite(NRE_PIN, LOW);
}

void bus_tx(uint8_t* buffer, uint8_t len) {
  tx_msg_buffer = buffer;
  tx_msg_len    = len;
}

void bus_thread(struct pt *pt) {
  static uint32_t last = 0;
  static uint8_t  writing = 0;
  
  PT_BEGIN(pt);

  // Initialize bus
  bus_init();
  
  while (1) {

    // Do we have something to send?
    if (tx_msg_len > 0) {

      // Wait until bus is unclaimed
      PT_WAIT_UNTIL(pt, digitalRead(BUS_IO_PIN) == HIGH);

      // Claim bus
      pinMode(BUS_IO_PIN, OUTPUT);
      digitalWrite(BUS_IO_PIN, LOW);

      // Set TX and wait for others to recognize
      bus_set_tx();
      last = millis();
      PT_WAIT_UNTIL(pt, millis() > (last + BUS_CLAIM_MS));

      // Write to serial one byte per 3 ms
      last = millis();
      writing = 0;
      while (writing < tx_msg_len) {
        Serial1.write(tx_msg_buffer, tx_msg_len); 
      }

      // Wait until all characters are written
      PT_WAIT_UNTIL(pt, Serial1.availableForWrite() == ard_buffer_size);
      tx_msg_len = 0;

      // Allow last byte to flush before releasing
      last = millis();
      PT_WAIT_UNTIL(pt, millis() > (last + BUS_CLEAN_MS));

      // Set RX and release bus
      bus_set_rx();
      pinMode(BUS_IO_PIN, INPUT_PULLUP);
    }

    // Nothing to do - just yield
    PT_YIELD(pt);
    
  }
  PT_END(pt);
}
