// libraries
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Serial.h>
#include <Wire.h>
#include <AS5600.h>
#include <MahonyAHRS.h>
#include <mbed.h>

// #include "PID_v1_bc.h"
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

//Ticker
mbed::Ticker timer;

// encoder object
AS5600 encoder;

// Mahony object
Mahony mahony;


/* variables timerInterrupt */
volatile bool timer_interrupt = false;


/* variables calculateArmAngle */
const float ENCODER_TO_ELBOW_OFFSET_DEGREES = 7.03125;
float encoder_degrees = 0;
uint16_t encoder_value = 0;


/* variables PIDControl */
unsigned long previous_PID_timestamp = 0;
unsigned long previous_encoder_timestamp = 0;
float elbow_angle = 0;
float previous_elbow_angle = 0;
#define FILTER_SIZE 12  // Number of values for moving average
double velocity_buffer[FILTER_SIZE] = {0};  // Circular buffer for velocity values
int velocity_index = 0;  // Index for buffer
double reference_velocity = 0;
double input_velocity = 0;
double output_velocity = 0; // moet dit 21000 worden?
double K_p = 25.0, K_i = 0.0, K_d = 0.0;  // Tuning parameters (pas aan voor optimale prestaties)
double error_velocity = 0; // difference between setpoint and processVariable  
double previous_error_velocity = 0; // error in previous iteration  
double PID_integral = 0; // integral of error  
double PID_derivative = 0; // derivative of error  
float average_velocity = 0; // average velocity of the motor
float motor_speed = 0; // motor speed
const float MAX_SPEED = 31400.0;
const float MIN_SPEED = 21000.0;


// variables setupEncoder
//none

// variables setup
//none

// variables loop
bool emergency_stop = false;


// const float gear_ratio = 1.0;  // Pas aan als je een overbrenging hebt


// bool isLeftProsthetic = false;

// float transformationMatrix[3][3]; // Will be set based on input
// float ax_offset = 0, ay_offset = 0, az_offset = 0; // initialize offset values
// float gx_offset = 0, gy_offset = 0, gz_offset = 0; // initialize offset values


void timerInterrupt(){
    if (BLUETOOTH) {
        timer_interrupt = true;
    }
}


float calculateElbowAngle(uint16_t encoder_value) {
    // const float ENCODER_TO_ARM_OFFSET_DEGREES = 7.03125; // kan dit globaal? voor het geval de waarde verandert

    float encoder_degrees = encoder_value * AS5600_RAW_TO_DEGREES;

    return encoder_degrees - ENCODER_TO_ELBOW_OFFSET_DEGREES;
}


void PIDControl(){
    if (BLUETOOTH) {
        // Determine time since last tick
        unsigned long PID_timestamp = micros();
        unsigned long delta_PID_timestamp = PID_timestamp - previous_PID_timestamp;
        previous_PID_timestamp = PID_timestamp;
            
        // Read encoder
        encoder_value = encoder.readAngle(); // read encoder value
        unsigned long encoder_timestamp = micros();
        elbow_angle = calculateElbowAngle(encoder_value);

        /* EMERGENCY BREAK */ // Deze verplaatsen zodat angular velocity meteen onder de enocder meting komt
        if (elbow_angle < 3 || 90 < elbow_angle) {
            disableMotor();
            emergency_stop = true;
            return;
        }

        // **Bereken snelheid in graden per seconde**
        float delta_elbow_angle = elbow_angle - previous_elbow_angle;  // Hoekverandering          
        float delta_encoder_timestamp = (encoder_timestamp - previous_encoder_timestamp) / 1000000.0; // Convert to seconds
        float raw_velocity = delta_elbow_angle / delta_encoder_timestamp; // Compute raw velocity

        previous_elbow_angle = elbow_angle; // update arm angle
        previous_encoder_timestamp = encoder_timestamp; // update time

        // Store value in moving average buffer
        velocity_buffer[velocity_index] = raw_velocity;
        velocity_index = (velocity_index + 1) % FILTER_SIZE; // Circular buffer

        // Compute moving average
        float velocity_sum = 0;
        for (int i = 0; i < FILTER_SIZE; i++) {
            velocity_sum += velocity_buffer[i];
        }
        input_velocity = velocity_sum / FILTER_SIZE;  // Smoothed velocity

        // PID-calculations           
        error_velocity = reference_velocity - input_velocity;  
        PID_integral += error_velocity;  
        PID_derivative = error_velocity - previous_error_velocity;  
        output_velocity = K_p * error_velocity + K_i * PID_integral + K_d * PID_derivative;  
        previous_error_velocity = error_velocity; 
        average_velocity = velocity_sum / FILTER_SIZE;

        // control motor
        motor_speed += output_velocity;
        motor_speed = constrain(motor_speed, MIN_SPEED, MAX_SPEED);
        turnStepsPerSecond((uint32_t) motor_speed, 0);  

        // Print data
        sendDataToPcf("Time: %6lu, dt: %lu, CEV: %d, RV: %6.2f, AV: %6.2f, E: %6.2f, I: %6.2f, D: %6.2f, MS: %6.2f",
            PID_timestamp,
            delta_PID_timestamp,
            encoder_value,
            raw_velocity,
            average_velocity,
            error_velocity,
            PID_integral,
            PID_derivative,
            motor_speed
        );
    }
}


void setupEncoder() {
    Serial.println("Beginning Encoder!");
    sendTextToPc("Beginning Encoder!");
    if (!encoder.begin()) {
        Serial.println("Failed to initialize Encoder!");
        sendTextToPc("Failed to initialize Encoder!");
        while (1);
    }
    Serial.println("Encoder initialized!");
    sendTextToPc("Encoder initialized!");
}

// setup, runs once
void setup() {
    Serial.begin(115200);
    Wire.begin(); delay(500); 
    setupBluetooth(); delay(500); 
    connectBluetoothToPc(); delay(500); 
    setupEncoder(); delay(500); 
    setupMotorControl(); delay(500); 
    Serial.println("Setup completed");
    sendTextToPcf("Setup completed after %d ms!", millis()); 
    delay(500);
    timer.attach(&timerInterrupt, std::chrono::milliseconds(20));
}

// continuous loop
void loop() {

    if(timer_interrupt){
        PIDControl();
        timer_interrupt = false;
    }

    if(emergency_stop){
        disableMotor();
        return;
    } 

    if (!encoder.begin()){
        disableMotor();
        emergency_stop = true;
        Serial.println("Encoder failure!");
        sendTextToPc("Encoder failure!");
        return;
    }
}