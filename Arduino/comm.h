#ifndef COMM_H
#define COMM_H

#define N_drive 3
#define N_mot 1
#define N_servo 2
#define N_us 1
#define N_lim 5
#define N_phot 0

struct status_output {
  int drive_mode;
  float drive_rpm[N_drive];
  float ypr[3];
  float US_dist[N_us];
  int lim_stat[N_lim];
  int phot_V[N_phot];
};

struct cmd_input {
  int drive_cmd;
  float drive_data[N_drive];
  bool US_enable;
  int mot_cmd[N_mot];
  int servo_cmd[N_servo];
};

bool read_command(cmd_input &command) {
  
  byte buff[sizeof(cmd_input)];

  if (Serial.available() > sizeof(cmd_input)) Serial.find('\r');
  else return 0;
  Serial.readBytes(buff, sizeof(cmd_input));
  memcpy(&command, buff, sizeof(cmd_input));
  while (Serial.available()) {
    Serial.read();
  }
  return 1;
}

byte buff[sizeof(status_output)];
void write_status(status_output out) {
  memcpy(buff, &out, sizeof(status_output));
  Serial.write(buff, sizeof(status_output));
  Serial.print('\n');
  Serial.flush();
}

status_output state;
cmd_input command;

#endif

