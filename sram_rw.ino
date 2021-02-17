// PORTD
const byte ADDR_0_3_MASK = 0b00001111; // out
const byte RESET_MASK = bit(4); // out, active low
const byte WE_MASK = bit(6); // out, active low
const byte CS_MASK = bit(7); // out, active low

// PORTE
const byte HALT_MASK = bit(6); // in, active low

// PORTF
const byte ADDR_4_7_MASK = 0b11110000; // out
const byte ADDR_8_9_MASK = 0b00000011; // out
const byte ADDR_4_9_MASK = ADDR_4_7_MASK | ADDR_8_9_MASK;

void enable_clock(const bool enable) {
  PORTC |= bit(6); //< set PC6 (OC3A) as output
  if (enable) {
    TCCR3A = bit(COM3A0); //< toggle OC3A on compare
  } else {
    TCCR3A = bit(COM3A1); //< clear OC3A on compare (hold clock pin low)
  }
  TCCR3B = bit(CS30) | bit(WGM32); //< no prescaling, CTC
  OCR3A = 1; // 4 MHz
}

void set_data_dir_out(const bool dir_out) {
  if (dir_out) {
    DDRB = 0xFF;
  } else {
    DDRB = 0; //< set port as input
    PORTB = 0; //< disable pull-ups (high-Z)
  }
}

void set_bus_dir_out(const bool dir_out) {
  if (dir_out) {
    set_data_dir_out(true);
    // Set address and WE/CS to output
    DDRD |= ADDR_0_3_MASK | WE_MASK | CS_MASK;
    DDRF |= ADDR_4_9_MASK;
  } else {
    set_data_dir_out(false);
    // Set address to high-Z and WE/CS to pull-up
    DDRD &= ~(ADDR_0_3_MASK | WE_MASK | CS_MASK);
    PORTD = (PORTD & ~ADDR_0_3_MASK) | WE_MASK | CS_MASK;
    DDRF &= ~ADDR_4_9_MASK;
    PORTF &= ~ADDR_4_9_MASK;
  }
}

void set_reset(const bool enable) {
  if (enable) {
    PORTD &= ~RESET_MASK;
  } else {
    PORTD |= RESET_MASK;
  }
}

bool read_halt() {
  return PINE & HALT_MASK;
}

void set_write_enable(const bool enable) {
  if (enable) {
    PORTD &= ~WE_MASK; //__asm__("cbi ?,?");
  } else {
    PORTD |= WE_MASK; //__asm__("sbi ?,?");
  }
}

void set_chip_select(const bool enable) {
  if (enable) {
    PORTD &= ~CS_MASK;
  } else {
    PORTD |= CS_MASK;
  }
}

void write_address(const uint16_t addr) {
  const byte low = addr & 0xFF;
  const byte high = addr >> 8;
  PORTD = (PORTD & ~ADDR_0_3_MASK) | (low & ADDR_0_3_MASK);
  PORTF = (PORTF & ~ADDR_4_9_MASK) | (low & ADDR_4_7_MASK) | (high & ADDR_8_9_MASK);
}

void write_data(const byte data) {
  set_chip_select(true);
  set_write_enable(true);

  PORTB = data;

  set_write_enable(false);
  set_chip_select(false);
}

byte read_data() {
  set_chip_select(true);

  // wait 2 cycles (need >70 ns after chip select), then sample data
  __asm__("nop");
  __asm__("nop");
  const byte data = PINB;

  set_chip_select(false);
  return data;
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  // Hold Z80 reset low
  set_reset(true);
  DDRD |= RESET_MASK;

  // Enable bus output
  set_chip_select(false);
  set_write_enable(false);
  set_bus_dir_out(true);

  // Set halt to active low input with pullup
  PORTD |= HALT_MASK;
  DDRE &= ~HALT_MASK;

  enable_clock(true);
}

void memwrite(uint16_t addr, byte data) {
  set_data_dir_out(true);
  write_address(addr);
  write_data(data);
}

byte memread(uint16_t addr) {
  set_data_dir_out(false);
  write_address(addr);
  return read_data();
}

void loop() {
  const int input = Serial.read();
  if (input == 'w') {
    // Write Serial byte stream to external memory
    for (uint16_t addr = 0; addr < 1024;) {
      const int input = Serial.read();
      if (input >= 0) {
        memwrite(addr++, input); //< increment addr
        if (input == '\n')
          break;
      }
    }
  } else if (input == 'r') {
    // Stream external memory to Serial
    for (uint16_t addr = 0; addr < 1024; ++addr) {
      const byte output = memread(addr);
      Serial.write(output);
      if (output == '\n' || output == 0)
        break;
    }
  } else if (input == 't') {
    bool fail = false;
    const byte patterns[] = {0xFF, 0xF0, 0x0F, 0xCC, 0x33, 0xAA, 0x55, 0x00};
    for (byte pattern : patterns) {
      for (uint16_t addr = 0; addr < 1024; ++addr) {
        memwrite(addr, pattern);
      }
      for (uint16_t addr = 0; addr < 1024; ++addr) {
        const byte data = memread(addr);
        const byte expected = pattern;
        if (data != expected) {
          Serial.print("FAIL: ");
          Serial.print(data, BIN);
          Serial.print(" != ");
          Serial.print(expected, BIN);
          Serial.print(" at ");
          Serial.println(addr, BIN);
          fail = true;
        }
      }
    }
    if (!fail) {
      Serial.println("PASS");
    }
  }
}
