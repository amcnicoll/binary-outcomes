// This is a lookup array from brightness to duty cycle
#define EXP_DIV_T   8192
uint8_t duty_curve[9] = {0, 2, 5, 10, 50, 100, 150, 200, 255};

// This interpolates the duty curve from a uint16_t brightness (from 0 to full)
uint8_t duty_lookup(uint16_t brightness) {
  uint16_t  low_i;
  uint16_t  low_y;
  uint16_t  hi_y;
  uint16_t  rem_b;

  low_i   = brightness >> 13;
  low_y   = duty_curve[low_i];
  hi_y    = duty_curve[low_i + 1];
  rem_b   = brightness - (low_i << 13);
  rem_b   = rem_b >> 5;
  
  return low_y + (((hi_y - low_y) * rem_b) >> 8);
}