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
  DDRC |= bit(6); //< set PC6 (OC3A) as output
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
    //set_data_dir_out(true);
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
  return !(PINE & HALT_MASK);
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

void dma_write(uint16_t addr, byte data) {
  set_data_dir_out(true);
  write_address(addr);
  write_data(data);
}

byte dma_read(uint16_t addr) {
  set_data_dir_out(false);
  write_address(addr);
  return read_data();
}

void dma_write_progmem_(const uint16_t addr, const byte data[] PROGMEM, const uint16_t size) {
  set_data_dir_out(true);
  set_chip_select(true);
  for (uint16_t i = 0; i < size; ++i) {
    write_address(addr + i);
    set_write_enable(true);
    PORTB = pgm_read_byte(data + i);
    set_write_enable(false);
  }
  set_chip_select(false);
}

#define dma_write_progmem(addr, data) dma_write_progmem_(addr, data, sizeof(data))

bool dma_verify_progmem_(const uint16_t addr, const byte data[] PROGMEM, const uint16_t size) {
  set_data_dir_out(false);
  set_chip_select(true);
  for (uint16_t i = 0; i < size; ++i) {
    write_address(addr + i);
    const byte expected = pgm_read_byte(data + i);
    const byte read = PINB;
    if (read != expected) {
      Serial.print("Data corrupt at ");
      Serial.print(addr);
      Serial.print(": expected ");
      Serial.print(expected);
      Serial.print(" but read ");
      Serial.println(read);
      return false;
    }
  }
  set_chip_select(false);
  return true;
}

#define dma_verify_progmem(addr, data) dma_verify_progmem_(addr, data, sizeof(data))

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  // Hold Z80 reset low
  DDRD |= RESET_MASK;
  set_reset(true);

  // Enable bus output
  set_bus_dir_out(true);
  set_chip_select(false);
  set_write_enable(false);

  // Set halt to active low input with pullup
  DDRE &= ~HALT_MASK;
  PORTD |= HALT_MASK;

  enable_clock(true);
}

const uint16_t DATA_ADDR = 512;
const uint16_t RESULT_ADDR = 640;
const uint16_t STACK_ADDR = 1024; // top of memory

const byte TEST_CODE[] PROGMEM = {
  [ 0] = 0x31,  // LD SP, STACK_ADDR
  [ 1] = STACK_ADDR & 0xFF,
  [ 2] = STACK_ADDR >> 8,
  [ 3] = 0x21,  // LD HL, DATA_ADDR
  [ 4] = DATA_ADDR & 0xFF,
  [ 5] = DATA_ADDR >> 8,
  [ 6] = 0x4E,  // LD C, (HL)
  [ 7] = 0x06,  // LD B, 0
  [ 8] = 0,     // ^
  [ 9] = 0x23,  // INC HL
  [10] = 0x11,  // LD DE, RESULT_ADDR
  [11] = RESULT_ADDR & 0xFF,
  [12] = RESULT_ADDR >> 8,
  [13] = 0xC5,  // PUSH BC
  [14] = 0xD5,  // PUSH DE
  [15] = 0xED,  // LDIR (copy BC bytes from HL to DE)
  [16] = 0xB0,  // ^
  [17] = 0xE1,  // POP HL (RESULT_ADDR to HL)
  [18] = 0xC1,  // POP BC
  [19] = 0x41,  // LD B, C (DJNZ uses B, not BC)
  [20] = 0x3E,  // LD A, 'z'    : loop
  [21] = 'z',   // ^
  [22] = 0xBE,  // CP (HL)
  [23] = 0x38,  // JR C  (if (HL) > 'z', goto next)
  [24] = (byte)(48 - 25),
  [25] = 0x7E,  // LD A, (HL)
  [26] = 0xFE,  // CP 'n'
  [27] = 'n',   // ^
  [28] = 0x30,  // JR NC, goto -= 13  (if (HL) >= 'n')
  [29] = (byte)(58 - 30),
  [30] = 0xFE,  // CP 'a'
  [31] = 'a',   // ^
  [32] = 0x30,  // JR NC, goto += 13  (if (HL) >= 'a')
  [33] = (byte)(52 - 34),
  [34] = 0x3E,  // LD A, 'Z'
  [35] = 'Z',   // ^
  [36] = 0xBE,  // CP (HL)
  [37] = 0x38,  // JR C, goto next  (if (HL) > 'Z')
  [38] = (byte)(48 - 39),
  [39] = 0x7E,  // LD A, (HL)
  [40] = 0xFE,  // CP 'N'
  [41] = 'N',   // ^
  [42] = 0x30,  // JR NC, goto -= 13  (if (HL) >= 'N')
  [43] = (byte)(58 - 44),
  [44] = 0xFE,  // CP 'A'
  [45] = 'A',   // ^
  [46] = 0x30,  // JR NC, goto += 13  (if (HL) >= 'A')
  [47] = (byte)(52 - 48),
  [48] = 0x23,  // INC HL   : next
  [49] = 0x10,  // DJNZ loop
  [50] = (byte)(20 - 51),
  [51] = 0x76,  // HALT
  [52] = 0x7E,  // LD A, (HL)  : += 13
  [53] = 0xC6,  // ADD A, 13
  [54] = 13,    // ^
  [55] = 0x77,  // LD (HL), A
  [56] = 0x18,  // JR to next
  [57] = (byte)(48 - 58),
  [58] = 0x7E,  // LD A, (HL)  : -= 13
  [59] = 0xD6,  // SUB A, 13
  [60] = 13,    // ^
  [61] = 0x77,  // LD (HL), A
  [62] = 0x18,  // JR to next
  [63] = (byte)(48 - 64),
};

const byte TEST_DATA[] PROGMEM = {
  [ 0] = 13,    // length of string
  [ 1] = 'U',   // super secret cypher encoding
  [ 2] = 'r',
  [ 3] = 'y',
  [ 4] = 'y',
  [ 5] = 'b',
  [ 6] = ',',
  [ 7] = ' ',
  [ 8] = 'J',
  [ 9] = 'b',
  [10] = 'e',
  [11] = 'y',
  [12] = 'q',
  [13] = '!',
};

void loop() {
  const int input = Serial.read();
  if (input == 'z') {
    Serial.print("Programming ");
    Serial.print(sizeof(TEST_CODE) + sizeof(TEST_DATA));
    Serial.println(" bytes...");
    set_reset(true);
    set_bus_dir_out(true);
    dma_write_progmem(0, TEST_CODE);
    dma_write_progmem(DATA_ADDR, TEST_DATA);

    Serial.println("Verifying...");
    if (!dma_verify_progmem(0, TEST_CODE) || !dma_verify_progmem(DATA_ADDR, TEST_DATA)) {
      return;
    }

    Serial.print("Running");
    set_bus_dir_out(false);
    set_reset(false);
    while (!read_halt()) {
      Serial.print('.');
    }

    set_reset(true);
    set_bus_dir_out(true);
    Serial.print("\nResult: ");
    char result[14];
    for (byte i = 0; i < 13; ++i)
      result[i] = dma_read(RESULT_ADDR + i);
    result[13] = 0;
    Serial.println(result);
  } else if (input == 't') {
    bool fail = false;
    const byte patterns[] = {0xFF, 0xF0, 0x0F, 0xCC, 0x33, 0xAA, 0x55, 0x00};
    for (byte pattern : patterns) {
      for (uint16_t addr = 0; addr < 1024; ++addr) {
        dma_write(addr, pattern);
      }
      for (uint16_t addr = 0; addr < 1024; ++addr) {
        const byte data = dma_read(addr);
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
