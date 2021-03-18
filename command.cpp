#include "command.hpp"

void handle_command_(char* input, const Command commands[], const byte length) {
  if (input == nullptr) {
    input = (char*)"";
  }

  for (byte i = 0; i < length; ++i) {
    if (strcmp(input, commands[i].command) == 0) {
      commands[i].function();
      return;
    }
  }

  Serial.println("?");
}
