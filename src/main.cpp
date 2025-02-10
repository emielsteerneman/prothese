// libraries
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Serial.h>
#include "stdint.h"
#include <Wire.h>
#include <AS5600.h>
#include <AccelStepper.h>

// define pins
#define MOTOR_STEP_PIN 3
#define MOTOR_DIR_PIN 4
#define MOTOR_ENABLE_PIN 5

// motor object
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN); 

// encoder object
AS5600 encoder;

// define states
enum Target {
    NOTHING,
    STRETCHED, // 10 degrees
    BENT // 80 degrees
};

enum State {
    HALTING,
    MOVING,
    REACHED,
    EMERGENCY_STOP
};

// mapping from enum to string for debugging
static const char* TARGETS[] = {
    "NOTHING",
    "STRETCHED",
    "BENT"
};

static const char* STATES[] = {
    "HALTING",
    "MOVING",
    "REACHED",
    "EMERGENCY_STOP"
};

// global variables 
const uint32_t LOOP_INTERVAL = 20; // in ms
const float MAX_SPEED = 17900.0; //7500.0
const float ACCELERATION = 50000.0; //100
const float ERROR_MARGIN_ANGLE = 2.0;
const long STEPS_PER_REV = 200*16;
uint8_t currentTarget = Target::NOTHING;
uint8_t currentState = State::HALTING;
float speed = 17900.0;


// BlueTooth service and characteristics
BLEService myService("12345678-1234-5678-1234-56789abcdef0"); // service UUID
// https://docs.arduino.cc/libraries/arduinoble/#BLECharacteristic%20Class
BLECharacteristic ArduinoToPc("12345678-1234-5678-1234-56789abcdef1", BLERead | BLENotify, 256); // read and notify UUID
BLEStringCharacteristic PcToArduino("12345678-1234-5678-1234-56789abcdef2", BLEWrite, 32); // write UUID

// data buffer
char send_buffer_string[256];

// setup functions
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

void setup_motor(){
    Serial.println("Beginning Motor!");

    stepper.setPinsInverted(false, true);
    stepper.setMaxSpeed(MAX_SPEED); 
    stepper.setAcceleration(ACCELERATION);

    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);

    Serial.println("Motor initialized!");
}

// encoder functions
float encoder_to_arm_angle(uint16_t encoder_value) {
    const float ENCODER_TO_ARM_OFFSET_DEGREES = 7.03125;

    float encoder_degrees = encoder_value * AS5600_RAW_TO_DEGREES;

    return encoder_degrees - ENCODER_TO_ARM_OFFSET_DEGREES;
}

float target_to_arm_angle(uint8_t target){
    if(target == Target::STRETCHED) 
        return 10;
    if(target == Target::BENT) 
        return 80;
    return 45;
}

// setup, runs once
void setup() {
    Serial.begin(115200);
    Wire.begin();

    delay(1000);
    setup_encoder();
    delay(1000);
    setup_imu();
    delay(1000);
    setup_motor();
    delay(1000);
    setup_bluetooth();

    Serial.println("Setup done!");
}

// continuous loop

void loop() {
    // Initialization variables
    float acceleration[3] = {0, 0, 0};
    float gyroscope[3] = {0, 0, 0};
    uint32_t timestamp_next_loop = 0;
    uint16_t current_encoder_value = 0;
    float current_arm_angle = 0.;
    float angle_error = 0;

    // Check if the Arduino is wirelessly connected to the PC via Bluetooth
    BLEDevice central = BLE.central();

    if (central) {
        Serial.println("Connected to PC!");

        // Timestamp
        timestamp_next_loop = millis() + LOOP_INTERVAL;

        // While the Bluetooth connection is active
        while (central.connected()) {

            // Read input from the PC/user
            if (PcToArduino.written()) {
                String received = PcToArduino.value();
                Serial.print("Received from central: ");
                Serial.println(received);

                if (received == "STRETCHED") {
                    currentTarget = Target::STRETCHED;
                    currentState = State::MOVING;
                    // stepper.moveTo(-999999);  // Set a new movement target
                } 
                else if (received == "BENT") {
                    currentTarget = Target::BENT;
                    currentState = State::MOVING;
                    // stepper.moveTo(999999);
                }
            }

            // CONTROL LOOP
            if (timestamp_next_loop <= millis()) {
                while (timestamp_next_loop <= millis()) {
                    timestamp_next_loop += LOOP_INTERVAL;
                }

                // Read IMU data
                IMU.readAcceleration(acceleration[0], acceleration[1], acceleration[2]);
                IMU.readGyroscope(gyroscope[0], gyroscope[1], gyroscope[2]);

                // Determine absolute encoder angle
                current_encoder_value = encoder.readAngle();
                current_arm_angle = encoder_to_arm_angle(current_encoder_value);

                // MOTOR CONTROL
                float target_angle = target_to_arm_angle(currentTarget);
                angle_error = target_angle - current_arm_angle;

                // If not at target, move motor
                if (currentTarget != Target::NOTHING && ERROR_MARGIN_ANGLE < fabs(angle_error)) {
                    // if (stepper.distanceToGo() == 0) {  // Set target only once
                    if (angle_error < 0) {
                        stepper.setSpeed(-1000);
                    } else {
                        stepper.setSpeed(1000);
                    }
                } 
                else {
                    currentTarget = Target::NOTHING;
                    currentState = State::REACHED;
                    stepper.stop();  // Gracefully stop motor
                }

                if(current_arm_angle < 7 || 83 < current_arm_angle){
                    currentState = State::EMERGENCY_STOP;
                    stepper.setSpeed(0.0f);
                    stepper.stop();
                }

                // Send data to the PC
                sprintf(send_buffer_string, "%s -> %s |DTG %d |E %.2f |A %+.2f %+.2f %+.2f |G %+.3f %+.3f %+.3f |° %d %.3f",
                    TARGETS[currentTarget], STATES[currentState],
                    stepper.distanceToGo(),
                    angle_error,
                    acceleration[0], acceleration[1], acceleration[2],
                    gyroscope[0], gyroscope[1], gyroscope[2],
                    current_encoder_value, current_arm_angle);
                ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));
            }

            // **Always run stepper!**
            stepper.run();  
        }
    }
}

// void loop() {

//     // initialization variables
//     float acceleration[3] = {0, 0, 0};
//     float gyroscope[3] = {0, 0, 0};
//     uint32_t timestamp_next_loop = 0;
//     uint16_t encoder_value = 0;
//     float angle_error = 0;

//    // String current_state = "HALTING";

//     // Check if the Arduino is wirelessly connected to the PC via bluetooth
//     BLEDevice central = BLE.central();

//     if (central) {
//         Serial.println("Connected to PC!");
        
//         // timestamp
//         timestamp_next_loop = millis() + LOOP_INTERVAL;

//         // While the bluetooth connection is active
//         while (central.connected()) {
            
//             // Read any input from the PC/user
//             if (PcToArduino.written()) {
//                 String received = PcToArduino.value();
//                 Serial.print("Received from central: ");
//                 Serial.println(received);
                
//                 if(received == "STRETCHED") {
//                     currentTarget = Target::STRETCHED;
//                     currentState = State::MOVING;
//                 }
//                 else if(received == "BENT") {
//                     currentTarget = Target::BENT;
//                     currentState = State::MOVING;
//                 }
//             }
//             if (stepper.distanceToGo() != 0) {
//                 stepper.run();
//             }

//             // CONTROL LOOP
//             if (timestamp_next_loop <= millis()) {
//                 while(timestamp_next_loop <= millis()) {
//                     timestamp_next_loop += LOOP_INTERVAL;
//                 }

//                 // read acc and gyr, assuming new data is available
//                 IMU.readAcceleration(acceleration[0], acceleration[1], acceleration[2]);
//                 IMU.readGyroscope(gyroscope[0], gyroscope[1], gyroscope[2]);

//                 // determime absolute encoder angle
//                 encoder_value = encoder.readAngle();
//                 float arm_angle = encoder_to_arm_angle(encoder_value);

//                 // MOTOR CONTROL
//                 float target_angle = target_to_arm_angle(currentTarget);
//                 angle_error = target_angle - arm_angle;
//                 if(currentTarget != Target::NOTHING &&  ERROR_MARGIN_ANGLE < fabs(angle_error)) {
//                     currentState = State::MOVING;
//                     if(angle_error < 0) {
//                         stepper.moveTo(-STEPS_PER_REV);
//                     } else {
//                         stepper.moveTo(STEPS_PER_REV);

//                     }
//                 }else{
//                     currentTarget = Target::NOTHING;
//                     currentState = State::REACHED;
//                     stepper.setSpeed(0.0f);
//                 }

//                 // Send data to the PC
//                 sprintf(send_buffer_string, "%s -> %s |M %.2f |A %+.2f %+.2f %+.2f |G %+.3f %+.3f %+.3f |E %d %.3f",
//                     TARGETS[currentTarget], STATES[currentState],
//                     angle_error,
//                     acceleration[0], acceleration[1], acceleration[2],
//                     gyroscope[0], gyroscope[1], gyroscope[2],
//                     encoder_value, arm_angle);
//                 ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));

//             }
//         }
//     }
// }