#ifndef BLUETOOTH_STUFF_H
#define BLUETOOTH_STUFF_H

#include <ArduinoBLE.h>

#define SEND_BUFFER_SIZE 256
#define RECEIVE_BUFFER_SIZE 32

// Make the variable BLUETOOTH available in other files
extern BLEDevice BLUETOOTH;

void setup_bluetooth();
void connect_bluetooth_to_pc();
bool pc_has_written();
String get_pc_input();
void send_text_to_pc(const char* string);
void send_text_to_pc_f(const char* format, ...);
void send_data_to_pc_f(const char* format, ...);


#endif // BLUETOOTH_STUFF_H