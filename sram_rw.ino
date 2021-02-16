const byte LOW_MASK  = 0b00111111; // PORTB
const byte HIGH_MASK = 0b00111100; // PORTD
const byte CS_MASK   = 0b01000000; // PORTD
const byte DATA_MASK = 0b00001111; // PORTC
const byte WE_MASK   = 0b00010000; // PORTC

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  // Assign output pins
  DDRB |= LOW_MASK;
  DDRC |= WE_MASK;
  DDRD |= HIGH_MASK | CS_MASK;

  // Set (disable) write enable and chip select
  PORTC |= WE_MASK;
  PORTD |= CS_MASK;
}

void memwrite(uint16_t addr, byte data) {
  // assign data pins as output
  DDRC |= DATA_MASK;

  // write address
  PORTD = (PORTD & ~HIGH_MASK) | ((addr >> 4) & HIGH_MASK);
  PORTB = (PORTB & ~LOW_MASK) | (addr & LOW_MASK);

  // clear (enable) chip select
  PORTD &= ~CS_MASK;

  // clear (enable) write enable
  PORTC &= ~WE_MASK; //__asm__("cbi 8,4\n\t");

  //PORTC = (PORTC & ~(WE_MASK | DATA_MASK)) | (data & DATA_MASK);
  PORTC = (PORTC & ~DATA_MASK) | (data & DATA_MASK);

  // disable write enable
  PORTC |= WE_MASK; //__asm__("sbi 8,4\n\t");

  // set (disable) chip select
  PORTD |= CS_MASK;
}

byte memread(uint16_t addr) {
  // assign data pins as input
  PORTC &= ~DATA_MASK; //< disables pull-ups in input mode
  DDRC &= ~DATA_MASK;

  // write address
  PORTD = (PORTD & ~HIGH_MASK) | ((addr >> 4) & HIGH_MASK);
  PORTB = (PORTB & ~LOW_MASK) | (addr & LOW_MASK);

  // clear (enable) chip select and wait 2 cycles (>70 ns)
  PORTD &= ~CS_MASK;
  __asm__("nop\n\t");
  __asm__("nop\n\t");

  byte data = PINC & DATA_MASK;

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
        memwrite(addr++, input >> 4); //< increment addr
        if (input == '\n')
          break;
      }
    }
  } else if (input == 'r') {
    // Stream external memory to Serial
    for (uint16_t addr = 0; addr < 1024; addr += 2) {
      const byte output = memread(addr + 1) << 4 | memread(addr);
      Serial.write(output);
      if (output == '\n' || output == 0)
        break;
    }
  } else if (input == 't') {
    const byte patterns[] = {0xF, 0x5, 0xA, 0x0};
    for (byte pattern : patterns) {
      for (uint16_t addr = 0; addr < 1024; ++addr) {
        memwrite(addr, (pattern ^ addr) & DATA_MASK);
      }
      for (uint16_t addr = 0; addr < 1024; ++addr) {
        const byte data = memread(addr);
        const byte expected = (pattern ^ addr) & DATA_MASK;
        if (data != expected) {
          Serial.print("FAIL: ");
          Serial.print(data, HEX);
          Serial.print(" != ");
          Serial.print(expected, HEX);
          Serial.print(" at ");
          Serial.println(addr);
          return;
        }
      }
    }
    Serial.println("PASS");
  }
}
