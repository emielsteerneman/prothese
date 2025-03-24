#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <ArduinoBLE.h>

#define SEND_BUFFER_SIZE 256
#define RECEIVE_BUFFER_SIZE 32

// Make the variable BLUETOOTH available in other files
extern BLEDevice BLUETOOTH;

void setupBluetooth();
void connectBluetoothToPc();
bool pcHasWritten();
String getPcInput();
void sendTextToPc(const char* string);
void sendTextToPcf(const char* format, ...);
void sendDataToPcf(const char* format, ...);


#endif // BLUETOOTH_H