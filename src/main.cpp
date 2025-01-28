// Arduino
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Serial.h>
#include <stdint.h>
#include <Wire.h>
#include <AS5600.h>

BLEService myService("12345678-1234-5678-1234-56789abcdef0");

// https://docs.arduino.cc/libraries/arduinoble/#BLECharacteristic%20Class
BLECharacteristic ArduinoToPc("12345678-1234-5678-1234-56789abcdef1", BLERead | BLENotify, 256);
BLEStringCharacteristic PcToArduino("12345678-1234-5678-1234-56789abcdef2", BLEWrite, 32);

// Encoder stuff
AS5600 encoder;
uint16_t ENCODER_MIN = 80;
uint16_t ENCODER_MAX = 1100;

const unsigned int LOOP_INTERVAL = 20;

char send_buffer_string[256];
uint8_t send_buffer_floats[256];

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

void setup_imu() {
    Serial.println("Beginning IMU!");
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }
}

void setup_encoder() {
    Serial.println("Beginning Encoder!");
    if (!encoder.begin()) {
        Serial.println("Failed to initialize Encoder!");
        while (1);
    }
    Serial.println("Encoder initialized!");
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    delay(5000);
    setup_encoder();
    delay(1000);
    setup_imu();
    delay(1000);
    setup_bluetooth();
    delay(1000);
}

/* ================================== */

float encoder_to_arm_angle(uint16_t encoder_value) {
    const float ENCODER_TO_ARM_OFFSET_DEGREES = 7.03125;

    float encoder_degrees = encoder_value * AS5600_RAW_TO_DEGREES;

    return encoder_degrees - ENCODER_TO_ARM_OFFSET_DEGREES;
}

void loop() {

    float acceleration[3] = {0, 0, 0};
    float gyroscope[3] = {0, 0, 0};

    // boolean imu_accel_read = false;
    // boolean imu_gyro_read = false;
    String current_state = "HALTING";

    // unsigned int imu_accel_counter = 0;
    // unsigned int imu_gyro_counter = 0;

    unsigned int time_next_loop = 0;

    uint16_t encoder_value = 0;

    // Check if the PC to connect via bluetooth
    BLEDevice central = BLE.central();

    // If the Arduino is wirelessly connected to the PC via bluetooth
    if (central) {
        Serial.println("Connected to PC!");
        
        time_next_loop = millis() + LOOP_INTERVAL;
        
        // While the bluetooth connection is active
        while (central.connected()) {
            
            // Read any input from the PC
            if (PcToArduino.written()) {
                String received = PcToArduino.value();
                Serial.print("Received from central: ");
                Serial.println(received);
                
                if(received == "START") {
                    current_state = "RUNNING";
                }else 
                if(received == "STOP") {
                    current_state = "HALTING";
                }
            }

            // if(IMU.accelerationAvailable()) {
            //     // IMU.readAcceleration(aX, aY, aZ);
            //     IMU.readAcceleration(acceleration[0], acceleration[1], acceleration[2]);
            //     imu_accel_counter++;
            //     imu_accel_read = true;
            // }
            // if (IMU.gyroscopeAvailable()) {
            //     IMU.readGyroscope(gyroscope[0], gyroscope[1], gyroscope[2]);
            //     imu_gyro_counter++;
            //     imu_gyro_read = true;
            // }
            
            if (time_next_loop < millis()) {
                while(time_next_loop < millis()) {
                    time_next_loop += LOOP_INTERVAL;
                }

                // This assumes that the IMU is always available
                IMU.readAcceleration(acceleration[0], acceleration[1], acceleration[2]);
                IMU.readGyroscope(gyroscope[0], gyroscope[1], gyroscope[2]);

                encoder_value = encoder.readAngle();
                float arm_angle = encoder_to_arm_angle(encoder_value);

                // Send IMU data to the PC
                sprintf(send_buffer_string, "%s |A %+.2f %+.2f %+.2f |G %+.2f %+.2f %+.2f |E %d %.2f",
                    current_state.c_str(),
                    acceleration[0], acceleration[1], acceleration[2],
                    gyroscope[0], gyroscope[1], gyroscope[2],
                    encoder_value, arm_angle);
                ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));

            }

            // Write periodic messages to the PC
            // if(imu_accel_read && imu_gyro_read) {
            //     sprintf(send_buffer_string, "|A %d %.2f %.2f %.2f |G %d %.2f %.2f %.2f",
            //         imu_accel_counter, acceleration[0], acceleration[1], acceleration[2],
            //         imu_gyro_counter, gyroscope[0], gyroscope[1], gyroscope[2]);
            //     ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));
            //     imu_accel_read = false; 
            //     imu_gyro_read = false;
            // }else if (imu_accel_read) {
            //     sprintf(send_buffer_string, "|A %d %.2f %.2f %.2f",
            //         imu_accel_counter, acceleration[0], acceleration[1], acceleration[2]);
            //     ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));
            //     imu_accel_read = false;
            // }else if (imu_gyro_read) {
            //     sprintf(send_buffer_string, "|G %d %.2f %.2f %.2f",
            //         imu_gyro_counter, gyroscope[0], gyroscope[1], gyroscope[2]);
            //     ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));
            //     imu_gyro_read = false;
            // }
            

            // if (millis() - lastSendTime > SEND_INTERVAL) {
            //     lastSendTime += SEND_INTERVAL;
            //     aX += 0.1;
            //     aY += 1;
            //     aZ -= 2;
                
            //     // copy float to char buffer
            //     memcpy(send_buffer_floats, &aX, sizeof(aX));
            //     memcpy(send_buffer_floats + sizeof(aX), &aY, sizeof(aY));
            //     memcpy(send_buffer_floats + sizeof(aX) + sizeof(aY), &aZ, sizeof(aZ));

            //     ArduinoToPc.writeValue(send_buffer_floats, 12);

            // }
        }
    }
}