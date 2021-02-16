const byte DATA_MASK = 0b11111111; // PORTB

const byte ADDR_0_3_MASK = 0b00001111; // PORTD
const byte CS_MASK = 0b10000000; // PORTD
const byte WE_MASK = 0b01000000; // PORTD

const byte ADDR_4_7_MASK = 0b11110000; // PORTF
const byte ADDR_8_9_MASK = 0b00000011; // PORTF
const byte ADDR_4_9_MASK = ADDR_4_7_MASK | ADDR_8_9_MASK;

void write_address(uint16_t addr) {
  const byte low = addr & 0xFF;
  const byte high = addr >> 8;
  PORTD = (PORTD & ~ADDR_0_3_MASK) | (low & ADDR_0_3_MASK);
  PORTF = (PORTF & ~ADDR_4_9_MASK) | (low & ADDR_4_7_MASK) | (high & ADDR_8_9_MASK);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  // Set (disable) write enable and chip select
  PORTD |= WE_MASK | CS_MASK;

  // Assign output pins
  DDRB = DATA_MASK; //DDRB |= DATA_MASK;
  DDRD |= ADDR_0_3_MASK | WE_MASK | CS_MASK;
  DDRF |= ADDR_4_9_MASK;
}

void memwrite(uint16_t addr, byte data) {
  // assign data pins as output
  DDRB = DATA_MASK; //DDRB |= DATA_MASK;

  write_address(addr);

  // clear (enable) chip select
  PORTD &= ~CS_MASK;

  // clear (enable) write enable
  PORTD &= ~WE_MASK; //__asm__("cbi ?,?\n\t");

  PORTB = data; //PORTB = (PORTB & ~DATA_MASK) | (data & DATA_MASK);
  // TODO need a nop?

  // disable write enable
  PORTD |= WE_MASK; //__asm__("sbi ?,?\n\t");

  // set (disable) chip select
  PORTD |= CS_MASK;
}

byte memread(uint16_t addr) {
  // assign data pins as input
  PORTB = 0;//~DATA_MASK; //PORTB &= ~DATA_MASK; //< disables pull-ups in input mode
  DDRB = 0;//~DATA_MASK; //DDRB &= ~DATA_MASK;

  write_address(addr);

  // clear (enable) chip select and wait 2 cycles (>70 ns)
  PORTD &= ~CS_MASK;
  __asm__("nop\n\t");
  __asm__("nop\n\t");

  const byte data = PINB;// & DATA_MASK;

  // set (disable) chip select
  PORTD |= CS_MASK;

  return data;
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
