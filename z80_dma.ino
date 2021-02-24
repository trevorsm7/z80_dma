#include "command.hpp"

#include <string.h>

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
    PORTD &= ~WE_MASK;
  } else {
    PORTD |= WE_MASK;
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

void dma_write_byte(uint16_t addr, byte data) {
  set_data_dir_out(true);
  write_address(addr);
  write_data(data);
}

byte dma_read_byte(uint16_t addr) {
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

void memtest() {
  bool fail = false;
  const byte patterns[] = {0xFF, 0xF0, 0x0F, 0xCC, 0x33, 0xAA, 0x55, 0x00};
  for (byte pattern : patterns) {
    for (uint16_t addr = 0; addr < 1024; ++addr) {
      dma_write_byte(addr, pattern);
    }
    for (uint16_t addr = 0; addr < 1024; ++addr) {
      const byte data = dma_read_byte(addr);
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

void loop() {
  static const Command commands[] = {
    Command { "test", memtest },
    Command { "rot13", run_rot13 },
  };

  String input = Serial.readString();
  input.trim();
  if (input.length() > 0) {
    Serial.println(input);
    char* token = strtok((char*)input.c_str(), " ");
    handle_command(token, commands);
  }
}
