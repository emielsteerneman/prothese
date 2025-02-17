#include "bluetooth_stuff.h"

#include <Serial.h>
#include <stdarg.h>

char send_buffer[SEND_BUFFER_SIZE] = {0};

BLEDevice BLUETOOTH;

// BlueTooth service and characteristics
BLEService myService("12345678-1234-5678-1234-56789abcdef0"); // service UUID
// https://docs.arduino.cc/libraries/arduinoble/#BLECharacteristic%20Class
BLECharacteristic       ArduinoToPc("12345678-1234-5678-1234-56789abcdef1", BLERead | BLENotify, SEND_BUFFER_SIZE); // read and notify UUID
BLEStringCharacteristic PcToArduino("12345678-1234-5678-1234-56789abcdef2", BLEWrite, RECEIVE_BUFFER_SIZE); // write UUID


void setup_bluetooth() {
    Serial.println("Beginning BLE!");
    if (!BLE.begin()) {
        Serial.println("Starting BLE failed!");
        while (1);
    }

    BLE.setLocalName("Prothese");
    myService.addCharacteristic(ArduinoToPc);
    myService.addCharacteristic(PcToArduino);
    BLE.addService(myService);

    String address = BLE.address();
    Serial.print("BLE address: ");
    Serial.println(address);

    BLE.advertise();
    Serial.println("Ready to connect!");
}

void connect_bluetooth_to_pc(){
    Serial.println("Waiting for the PC to connect to the Arduino Bluetooth...");
    while(!BLUETOOTH){
        BLUETOOTH = BLE.central();
    }
    while(!BLUETOOTH.connected());
    Serial.println("PC connected to the Arduino Bluetooth!");

    // This is needed because for some reason, the first few writes to the PC are not received.. This keeps writing
    // until the PC has received the message and responds. We then know that the PC is ready to receive messages.
    while(!PcToArduino.written()){
        delay(200);
        while(!BLUETOOTH.connected());
        send_text_to_pc("INIT");
        Serial.print(".. Still waiting\r");
    }
}

bool pc_has_written(){
    return PcToArduino.written();
}

String get_pc_input(){
    return PcToArduino.value();
}

void send_text_to_pc(const char* string){
    sprintf(send_buffer, "TEXT%s", string);
    ArduinoToPc.writeValue(send_buffer, sizeof(send_buffer));
}

void send_text_to_pc_f(const char* format, ...){
    // Start by writing "TEXT" into the buffer
    sprintf(send_buffer, "TEXT");
    // Do all the other magic string stuff that places the formatted string into the buffer
    va_list args;
    va_start(args, format);
    vsnprintf(send_buffer + 4, sizeof(send_buffer) - 4, format, args);
    va_end(args);
    // Send the buffer to the PC via Bluetooth
    ArduinoToPc.writeValue(send_buffer, sizeof(send_buffer));
}

void send_data_to_pc_f(const char* format, ...){
    // Do all the magic string stuff that places the formatted string into the buffer
    va_list args;
    va_start(args, format);
    vsnprintf(send_buffer, sizeof(send_buffer), format, args);
    va_end(args);
    // Send the buffer to the PC via Bluetooth
    ArduinoToPc.writeValue(send_buffer, sizeof(send_buffer));
}