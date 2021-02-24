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
    byte len = dma_read_byte(DATA_ADDR);
    char result[len + 1];
    for (byte i = 0; i < len; ++i)
      result[i] = dma_read_byte(RESULT_ADDR + i);
    result[len] = 0;
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
      byte len = strlen(message);
      dma_write_byte(DATA_ADDR, len);
      for (byte i = 0; i < len; ++i) {
        dma_write_byte(DATA_ADDR + 1 + i, message[i]);
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
