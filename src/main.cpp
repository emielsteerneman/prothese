// libraries
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Serial.h>
#include <Wire.h>
#include <AS5600.h>
#include <MahonyAHRS.h>


#include "PID_v1_bc.h"
#include "stdint.h"
#include "IMUCalibration.h"
#include "bluetooth.h"
#include "motor_control.h"

// define pins
#define MS1_PIN 8
#define MS2_PIN 7
#define MOTOR_STEP_PIN 3
#define MOTOR_DIR_PIN 2
#define MOTOR_ENABLE_PIN 9

// encoder object
AS5600 encoder;

// Mahony object
Mahony mahony;

// moving average filter
#define FILTER_SIZE 5  // Number of values for moving average

float velocity_buffer[FILTER_SIZE] = {0};  // Circular buffer for velocity values
int velocity_index = 0;  // Index for buffer



// *PID Variabelen*
double setpoint, input, output;
double Kp = 5.0, Ki = 0.5, Kd = 0.2;  // Tuning parameters (pas aan voor optimale prestaties)
PID motorPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// **Tijd & Encoder Variabelen**
unsigned long lastTime = 0;
float lastAngle = 0;
const float gear_ratio = 1.0;  // Pas aan als je een overbrenging hebt


// global variables 
const uint32_t LOOP_INTERVAL = 20; // in ms
const int numReadings = 150;
const int numRounds = 40;    // Number of rounds to store quaternion values
const float MAX_SPEED = 31400.0;//17900.0;
const float MIN_SPEED = 21000.0;
// const float ACCELERATION = 50000.0;//50000.0; //100
const float ERROR_MARGIN_ANGLE = 0.5;
// const float STEPS_PER_DEGREE = 10666.67;
const float eta = 0.005f;    // Tolerance value for floating-point comparison
const float epsilon = 0.01; // Threshold for determining stability
bool isLeftProsthetic = false;
bool stable = false;           // Flag to determine if values are stable for stability check
int roundCount = 0;            // Counter for rounds for stability check
float transformationMatrix[3][3]; // Will be set based on input
float ax_offset = 0, ay_offset = 0, az_offset = 0; // initialize offset values
float gx_offset = 0, gy_offset = 0, gz_offset = 0; // initialize offset values
float q0Old = 0, q1Old = 0, q2Old = 0, q3Old = 0; // initialise previous value for stability check
float accValues[numRounds][3]; // Stores ax, ay, az for stability check
float gyrValues[numRounds][3]; // Stores gx, gy, gz for stability check
float target_arm_angle = 10;
bool emergency_stop = false;

// variables for algorithm 1
bool gxTriggered = false;
unsigned long gxTriggerTime = 0; // Stores the last time gx was triggered
unsigned long rollTriggerTime = 0; // Stores the last time roll was triggered
// const unsigned long gxCooldownPeriod = 500; // Cooldown period in milliseconds
const unsigned long rollCooldownPeriod = 1000; // Cooldown period in milliseconds

// variables for algorithm 2
bool gxEnabled = false;
const unsigned long gxCooldownPeriod = 1000; // Cooldown period in milliseconds // also used in algorithm 3

// variables for algorithm 3
unsigned long omegaXFlexTriggerTime = 0;
unsigned long omegaXExtendTriggerTime = 0;

// variables for test run
int motor_run_counter = 0;


void setup_encoder() {
    Serial.println("Beginning Encoder!");
    send_text_to_pc("Beginning Encoder!");
    if (!encoder.begin()) {
        Serial.println("Failed to initialize Encoder!");
        send_text_to_pc("Failed to initialize Encoder!");
        while (1);
    }
    Serial.println("Encoder initialized!");
    send_text_to_pc("Encoder initialized!");
}

float encoder_to_arm_angle(uint16_t encoder_value) {
    const float ENCODER_TO_ARM_OFFSET_DEGREES = 7.03125;

    float encoder_degrees = encoder_value * AS5600_RAW_TO_DEGREES;

    return encoder_degrees - ENCODER_TO_ARM_OFFSET_DEGREES;
}

// setup, runs once
void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(500); 
    setup_bluetooth();
    delay(500); 
    connect_bluetooth_to_pc();
    delay(500); 
    setup_encoder();
    delay(500); 
    setup_motor_control();
    delay(500); 
    setpoint = 8.5;  // Gewenste snelheid in stappen per seconde
    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(MIN_SPEED, MAX_SPEED);  
    send_text_to_pc_f("Setup completed after %d ms!", millis());

}

// continuous loop
void loop() {
    uint32_t timestamp_next_loop = 0;
    uint16_t current_encoder_value = 0;
    float current_arm_angle = 0.;

    if (BLUETOOTH) {
        timestamp_next_loop = millis() + LOOP_INTERVAL;

        while (!emergency_stop) {
            unsigned long now = millis();
            if (timestamp_next_loop <= millis()) {
                while (timestamp_next_loop <= millis()) {
                    timestamp_next_loop += LOOP_INTERVAL;
                }
                
                if (!encoder.begin()){
                    disable_motor();
                    emergency_stop = true;
                    Serial.println("Encoder failure!");
                    send_text_to_pc("Encoder failure!");
                    break;
                }
                
                 current_encoder_value = encoder.readAngle();
                 current_arm_angle = encoder_to_arm_angle(current_encoder_value);

                /* EMERGENCY BREAK */
                 if (current_arm_angle < 5 || 88 < current_arm_angle) {
                     disable_motor();
                     emergency_stop = true;
                     break;
                 }


                    // Read encoder
                    // current_encoder_value = encoder.readAngle();
                    // current_arm_angle = encoder_to_arm_angle(current_encoder_value);
                    float deltaAngle = current_arm_angle - lastAngle;  // Hoekverandering
                    lastAngle = current_arm_angle;
                    
                    // **Bereken snelheid in graden per seconde**
                    float deltaTime = (now - lastTime) / 1000.0; // Convert to seconds
                    // input = (deltaAngle * gear_ratio) / deltaTime; // Steps per second
                    lastTime = now;
                    // Compute raw velocity
                    float raw_velocity = (deltaAngle * gear_ratio) / deltaTime;

                    // Store value in moving average buffer
                    velocity_buffer[velocity_index] = raw_velocity;
                    velocity_index = (velocity_index + 1) % FILTER_SIZE; // Circular buffer

                    // Compute moving average
                    float sum = 0;
                    for (int i = 0; i < FILTER_SIZE; i++) {
                        sum += velocity_buffer[i];
                    }
                    input = sum / FILTER_SIZE;  // Smoothed velocity


                // current_encoder_value = encoder.readAngle();
                // current_arm_angle = encoder_to_arm_angle(current_encoder_value);
                // float deltaAngle = current_arm_angle - lastAngle;  // Hoekverandering
                // lastAngle = current_arm_angle;
                
                // **Bereken snelheid in stappen per seconde**
                // float deltaTime = (now - lastTime) / 1000.0; // Convert to seconds
                // input = (deltaAngle * gear_ratio) / deltaTime; // Steps per second
                // lastTime = now;


                // **PID-berekening uitvoeren**
                motorPID.Compute();
                uint32_t motorSpeed = constrain(output, MIN_SPEED, MAX_SPEED); // Limit to motor max speed  
                turn_steps_per_second(motorSpeed, 0);  // Set motor speed
                

                // send_data_to_pc_f("Setpoint %f |SPEED: %.2f |Output %f | Emergency %d",

                
                unsigned long timestamp = millis();

                send_data_to_pc_f("%lu,%d,%.2f",
                // /* L */ setpoint, 
                /* E */ 
                timestamp, 
                motorSpeed,
                input);
                // /* A */ output,
                // /* P */ emergency_stop);
            }

        } // while !emergency_stop
}     // if (bluetooth)
    else {
        Serial.println("NOT BLUETOOTH CONNECTED!");
    }
} // loop()