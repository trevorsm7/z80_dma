namespace rot13 {
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
    [ 6] = 0x11,  // LD DE, RESULT_ADDR
    [ 7] = RESULT_ADDR & 0xFF,
    [ 8] = RESULT_ADDR >> 8,
    [ 9] = 0x3E,  // LD A, 0
    [10] = 0,     // ^
    [11] = 0x01,  // LD BC, 0
    [12] = 0,     // ^
    [13] = 0,     // ^

    // Count bytes in string at DATA_ADDR
    [14] = 0xE5,  // PUSH HL (DATA_ADDR)
    [15] = 0xED,  // CPIR (loop until A==(HL++))
    [16] = 0xB1,  // ^
    [17] = 0xE1,  // POP HL (DATA_ADDR)
    [18] = 0x91,  // SUB C
    [19] = 0x4F,  // LD C, A (BC = negated count from CPIR)
    [20] = 0x06,  // LD B, 0
    [21] = 0x00,  // ^

    // Copy string from DATA_ADDR to RESULT_ADDR
    [22] = 0xD5,  // PUSH DE (RESULT_ADDR)
    [23] = 0xED,  // LDIR ((HL++) -> (DE++), BC--)
    [24] = 0xB0,  // ^
    [25] = 0xE1,  // POP HL (RESULT_ADDR)
    [26] = 0x47,  // LD B, A (B = negated count from CPIR)

    // ROT13 encode/decode character at HL
    [27] = 0x3E,  // LD A, 'z'
    [28] = 'z',   // ^
    [29] = 0xBE,  // CP (HL)
    [30] = 0x38,  // JR C  (if (HL) > 'z', goto next)
    [31] = (byte)(55 - 32),
    [32] = 0x7E,  // LD A, (HL)
    [33] = 0xFE,  // CP 'n'
    [34] = 'n',   // ^
    [35] = 0x30,  // JR NC  (if (HL) >= 'n', goto -= 13)
    [36] = (byte)(65 - 37),
    [37] = 0xFE,  // CP 'a'
    [38] = 'a',   // ^
    [39] = 0x30,  // JR NC  (if (HL) >= 'a', goto += 13)
    [40] = (byte)(59 - 41),
    [41] = 0x3E,  // LD A, 'Z'
    [42] = 'Z',   // ^
    [43] = 0xBE,  // CP (HL)
    [44] = 0x38,  // JR C  (if (HL) > 'Z', goto next)
    [45] = (byte)(55 - 46),
    [46] = 0x7E,  // LD A, (HL)
    [47] = 0xFE,  // CP 'N'
    [48] = 'N',   // ^
    [49] = 0x30,  // JR NC  (if (HL) >= 'N', goto -= 13)
    [50] = (byte)(65 - 51),
    [51] = 0xFE,  // CP 'A'
    [52] = 'A',   // ^
    [53] = 0x30,  // JR NC  (if (HL) >= 'A', goto += 13)
    [54] = (byte)(59 - 55),

    // Ready next character
    [55] = 0x23,  // INC HL
    [56] = 0x10,  // DJNZ loop
    [57] = (byte)(27 - 58),
    [58] = 0x76,  // HALT

    // Add 13 to character
    [59] = 0x7E,  // LD A, (HL)
    [60] = 0xC6,  // ADD A, 13
    [61] = 13,    // ^
    [62] = 0x77,  // LD (HL), A
    [63] = 0x18,  // JR to next
    [64] = (byte)(55 - 65),

    // Sub 13 from character
    [65] = 0x7E,  // LD A, (HL)
    [66] = 0xD6,  // SUB A, 13
    [67] = 13,    // ^
    [68] = 0x77,  // LD (HL), A
    [69] = 0x18,  // JR to next
    [70] = (byte)(55 - 71),
  };
  
  const byte TEST_DATA[] PROGMEM = "Uryyb, Jbeyq!";

  void run() {
    Serial.print("Running");
    set_bus_dir_out(false);
    set_reset(false);
    while (!read_halt()) {
      Serial.print('.');
    }

    set_reset(true);
    set_bus_dir_out(true);
    Serial.print("\nResult: ");
    char result[128];
    for (byte i = 0; i < 128; ++i) {
      byte read = dma_read_byte(RESULT_ADDR + i);
      result[i] = read;
      if (read == 0)
        break;
    }
    Serial.println(result);
  }

  void program() {
    Serial.print("Programming ");
    Serial.print(sizeof(TEST_CODE) + sizeof(TEST_DATA));
    Serial.println(" bytes...");
    set_reset(true);
    set_bus_dir_out(true);
    dma_write_progmem(0, TEST_CODE);
    dma_write_progmem(DATA_ADDR, TEST_DATA);

    Serial.println("Verifying...");
    if (!dma_verify_progmem(0, TEST_CODE)
      || !dma_verify_progmem(DATA_ADDR, TEST_DATA)) {
      return;
    }

    run();
  }

  void input() {
    char* message = strtok(nullptr, "");
    if (message != nullptr) {
      for (byte i = 0; i < 128; ++i) {
        dma_write_byte(DATA_ADDR + i, message[i]);
        if (message[i] == 0)
          break;
      }
      run();
    } else {
      Serial.println("Expected message");
    }
  }
}

void run_rot13() {
  static const Command commands[] = {
    Command { "program", rot13::program },
    Command { "run", rot13::run },
    Command { "input", rot13::input },
  };

  char* token = strtok(nullptr, " ");
  handle_command(token, commands);
}
