#pragma once

#include "Arduino.h"

struct Command {
  const char* command;
  void (*function)();
};

void handle_command_(char* input, const Command commands[], const byte length);

#define handle_command(input, commands) \
  handle_command_(input, commands, sizeof(commands) / sizeof(commands[0]))
