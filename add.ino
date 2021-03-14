namespace add {
  const uint16_t OP1_ADDR = 512;
  const uint16_t OP2_ADDR = 513;
  const uint16_t RESULT_ADDR = 514;
  
  const byte TEST_CODE[] PROGMEM = {
    [ 0] = 0x3A,  // LD A, (OP2_ADDR)
    [ 1] = OP2_ADDR & 0xFF,
    [ 2] = OP2_ADDR >> 8,
    [ 3] = 0x47,  // LD B, A
    [ 4] = 0x3A,  // LD A, (OP1_ADDR)
    [ 5] = OP1_ADDR & 0xFF,
    [ 6] = OP1_ADDR >> 8,
    [ 7] = 0x80,  // ADD A, B
    [ 8] = 0x32,  // LD (RESULT_ADDR), A
    [ 9] = RESULT_ADDR & 0xFF,
    [10] = RESULT_ADDR >> 8,
    [11] = 0x76,  // HALT
  };

  void run(byte op1, byte op2) {
    set_reset(true);
    set_bus_dir_out(true);
    set_data_dir_out(true);

    Serial.println("Programming...");
    dma_write_progmem(0, TEST_CODE);
    dma_write_byte(OP1_ADDR, op1);
    dma_write_byte(OP2_ADDR, op2);

    Serial.print("Running");
    set_bus_dir_out(false);
    set_reset(false);
    while (!read_halt()) {
      Serial.print('.');
    }

    set_reset(true);
    set_bus_dir_out(true);
    set_data_dir_out(false);
    byte result = dma_read_byte(RESULT_ADDR);
    Serial.print("\nResult: ");
    Serial.println(result);
  }
}

void run_add() {
  char* op1 = strtok(nullptr, " ");
  char* op2 = strtok(nullptr, " ");
  if (op1 == nullptr || op2 == nullptr) {
    Serial.println("Expected two operands");
    return;
  }
  add::run(atoi(op1), atoi(op2));
}
