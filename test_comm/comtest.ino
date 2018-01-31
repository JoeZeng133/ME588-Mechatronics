#include "comm.h"

unsigned long prevtime_com = 0;
const unsigned long update_freq_com = 10;

status_output state;
cmd_input command;

void setup() {
  Serial.begin(115200);
  state.phot_V[0] = 0;
  Serial.print(sizeof(status_output));
}

void loop() {

  if (millis() - prevtime_com > 1000/update_freq_com) {
    if (read_command(command)) {
      write_status(state);
    }
  }
}
