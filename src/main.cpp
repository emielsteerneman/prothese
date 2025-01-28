// Arduino
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Serial.h>
#include <stdint.h>
#include <Wire.h>
#include <AS5600.h>
#include <AccelStepper.h>

enum Target {
    NOTHING,
    STRETCHED, // 0 degrees
    BENT // 90 degrees
};

enum State {
    HALTING,
    MOVING,
    REACHED
};

static const char* TARGETS[] = {
    "NOTHING",
    "STRETCHED",
    "BENT"
};

static const char* STATES[] = {
    "HALTING",
    "MOVING",
    "REACHED"
};

// Encoder stuff
AS5600 encoder;

// Motor stuff
#define MOTOR_STEP_PIN 3
#define MOTOR_DIR_PIN 4
#define MOTOR_ENABLE_PIN 5
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN);


// Control stuff
const unsigned int LOOP_INTERVAL = 20;
uint8_t CURRENT_TARGET = Target::NOTHING;
uint8_t CURRENT_STATE = State::HALTING;

// Bluetooth stuff
BLEService myService("12345678-1234-5678-1234-56789abcdef0");
// https://docs.arduino.cc/libraries/arduinoble/#BLECharacteristic%20Class
BLECharacteristic ArduinoToPc("12345678-1234-5678-1234-56789abcdef1", BLERead | BLENotify, 256);
BLEStringCharacteristic PcToArduino("12345678-1234-5678-1234-56789abcdef2", BLEWrite, 32);

// Log stuff
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

void setup_motor(){
    Serial.println("Beginning Motor!");

    stepper.setPinsInverted(false, true);
    stepper.setMaxSpeed(7500.0);
    stepper.setAcceleration(100.0);

    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);

    Serial.println("Motor initialized!");
}

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

    

    // float speedtogoat = 7500.0f;
    // while(1){
    //     stepper.setSpeed(speedtogoat);
    //     for(int i = 0; i < 100000; i++)
    //     {
    //         stepper.runSpeed();
    //         // delayMicroseconds(1);
    //     }
    //     speedtogoat = -speedtogoat;
    // }

}

/* ================================== */

float encoder_to_arm_angle(uint16_t encoder_value) {
    const float ENCODER_TO_ARM_OFFSET_DEGREES = 7.03125;

    float encoder_degrees = encoder_value * AS5600_RAW_TO_DEGREES;

    return encoder_degrees - ENCODER_TO_ARM_OFFSET_DEGREES;
}

float target_to_arm_angle(uint8_t target){
    if(target == Target::STRETCHED) return 10;
    if(target == Target::BENT) return 80;
    return 0;
}

void loop() {

    float acceleration[3] = {0, 0, 0};
    float gyroscope[3] = {0, 0, 0};

    String current_state = "HALTING";

    unsigned int time_next_loop = 0;

    uint16_t encoder_value = 0;
    float angle_error = 0;

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
                
                if(received == "STRETCHED") {
                    CURRENT_TARGET = Target::STRETCHED;
                    CURRENT_STATE = State::MOVING;
                }else
                if(received == "BENT") {
                    CURRENT_TARGET = Target::BENT;
                    CURRENT_STATE = State::MOVING;
                }
            }

            stepper.runSpeed();
            // delayMicroseconds(10);

            // CONTROL LOOP
            if (time_next_loop < millis()) {
                while(time_next_loop < millis()) {
                    time_next_loop += LOOP_INTERVAL;
                }

                // This assumes that the IMU is always available
                IMU.readAcceleration(acceleration[0], acceleration[1], acceleration[2]);
                IMU.readGyroscope(gyroscope[0], gyroscope[1], gyroscope[2]);

                encoder_value = encoder.readAngle();
                float arm_angle = encoder_to_arm_angle(encoder_value);

                // MOTOR CONTROL
                float target_angle = target_to_arm_angle(CURRENT_TARGET);
                angle_error = target_angle - arm_angle;
                if(CURRENT_TARGET != Target::NOTHING &&  2 < fabs(angle_error)) {
                    CURRENT_STATE = State::MOVING;
                    if(angle_error < 0) {
                        stepper.setSpeed(-7500.0f);
                    } else {
                        stepper.setSpeed(7500.0f);
                    }
                }else{
                    CURRENT_STATE = State::REACHED;
                    CURRENT_TARGET = Target::NOTHING;
                    stepper.setSpeed(0.0f);
                }

                // Send IMU data to the PC
                sprintf(send_buffer_string, "%s -> %s |M %.2f |A %+.2f %+.2f %+.2f |G %+.3f %+.3f %+.3f |E %d %.3f",
                    TARGETS[CURRENT_TARGET], STATES[CURRENT_STATE],
                    angle_error,
                    acceleration[0], acceleration[1], acceleration[2],
                    gyroscope[0], gyroscope[1], gyroscope[2],
                    encoder_value, arm_angle);
                ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));

            }

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