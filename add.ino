namespace add {
  const uint16_t OP1_ADDR = 512;
  const uint16_t OP2_ADDR = 513;
  const uint16_t RESULT_ADDR = 514;
  
  const byte BYTE_CODE[] PROGMEM = {
    [0] = 0x21, // LD HL, OP1_ADDR
    [1] = OP1_ADDR & 0xFF,
    [2] = OP1_ADDR >> 8,
    [3] = 0x7E, // LD A, (HL) ; A = (OP1)
    [4] = 0x23, // INC HL     ; HL = OP2_ADDR
    [5] = 0x86, // ADD (HL)   ; A = OP1 + (OP2)
    [6] = 0x23, // INC HL     ; HL = RESULT_ADDR
    [7] = 0x77, // LD (HL), A ; (RESULT) = OP1 + OP2
    [8] = 0x76, // HALT
  };

  void run(byte op1, byte op2) {
    set_reset(true);
    set_bus_dir_out(true);
    set_data_dir_out(true);

    Serial.println("Programming...");
    dma_write_progmem(0, BYTE_CODE);
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
