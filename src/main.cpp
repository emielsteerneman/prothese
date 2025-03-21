// libraries
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Serial.h>
#include <Wire.h>
#include <AS5600.h>
#include <MahonyAHRS.h>
#include <mbed.h>

#include "PID_v1_bc.h"
#include "stdint.h"
#include "IMUCalibration.h"
#include "bluetooth.h"
#include "motor_control.h"
// #include "mbed.h" //waarom moet deze met ""?

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

// moving average filter
#define FILTER_SIZE 12  // Number of values for moving average
double velocity_buffer[FILTER_SIZE] = {0};  // Circular buffer for velocity values
int velocity_index = 0;  // Index for buffer

// **Tijd & Encoder Variabelen**
// unsigned long lastTime = 0;
unsigned long previous_time_encoder_measured = 0;
float lastAngle = 0;
// const float gear_ratio = 1.0;  // Pas aan als je een overbrenging hebt


// global variables 
// const uint32_t LOOP_INTERVAL = 20000; // in micros
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




// *PID Variabelen*
double setpoint = 0;
double input = 0;
double output = 21000;

double Kp = 25.0, Ki = 0.0, Kd = 0.0;  // Tuning parameters (pas aan voor optimale prestaties)
double inputError = 0; // difference between setpoint and processVariable  
double previousError = 0; // error in previous iteration  
double PIDintegral = 0; // integral of error  
double PIDderivative = 0; // derivative of error  
float averageVelocity = 0; // average velocity of the motor
float motorSpeed = 21000; // motor speed


/* EMIEL CODE HERE. These will all be updated below */
// float SetPoint = 10.;
// float averageVelocity = 0.;
// float myError = 0.;
// float myNewError = 0.;
// float myOldError = 0.;
// float myMotorSpeed = 31400.;
// float myIntegral = 0.;
// float myDerivative = 0.;
/* ---- */

// uint32_t loop_counter = 0;
// uint32_t looooops_skipped = 0;
// int32_t dt_remainder = 0;

uint16_t current_encoder_value = 0;
float current_arm_angle = 0.;

volatile bool timerInterruption = false;

// unsigned long timer_test_time_previous_tick = 0;
// unsigned long timer_test_delay = 1;

void timer_test(){
    if (BLUETOOTH) {
        timerInterruption = true;
    }
}


float encoder_to_arm_angle(uint16_t encoder_value) {
    const float ENCODER_TO_ARM_OFFSET_DEGREES = 7.03125;

    float encoder_degrees = encoder_value * AS5600_RAW_TO_DEGREES;

    return encoder_degrees - ENCODER_TO_ARM_OFFSET_DEGREES;
}

unsigned long pid_control_time_previous_tick = 0;

void PID_control(){
    if (BLUETOOTH) {
        unsigned long now = micros();
        current_encoder_value = encoder.readAngle(); // read encoder value
        unsigned long dt = now - pid_control_time_previous_tick;
        pid_control_time_previous_tick = now;
            
        // Read encoder
        unsigned long time_encoder_measured = micros();
        current_arm_angle = encoder_to_arm_angle(current_encoder_value);

        /* EMERGENCY BREAK */
        if (current_arm_angle < 3 || 90 < current_arm_angle) {
            disable_motor();
            emergency_stop = true;
            return;
        }

        // **Bereken snelheid in graden per seconde**
        float deltaAngle = current_arm_angle - lastAngle;  // Hoekverandering          
        float deltaTime = (time_encoder_measured - previous_time_encoder_measured) / 1000000.0; // Convert to seconds
        float raw_velocity = deltaAngle / deltaTime; // Compute raw velocity

        lastAngle = current_arm_angle; // update arm angle
        previous_time_encoder_measured = time_encoder_measured; // update time

        // Store value in moving average buffer
        velocity_buffer[velocity_index] = raw_velocity;
        velocity_index = (velocity_index + 1) % FILTER_SIZE; // Circular buffer

        // Compute moving average
        float sum = 0;
        for (int i = 0; i < FILTER_SIZE; i++) {
            sum += velocity_buffer[i];
        }
        input = sum / FILTER_SIZE;  // Smoothed velocity

        // **PID-berekening uitvoeren**            
        /* EMY CODE HERE */
        inputError = setpoint - input;  
        PIDintegral += inputError;  
        PIDderivative = inputError - previousError;  
        output = Kp * inputError + Ki * PIDintegral + Kd * PIDderivative;  
        previousError = inputError; 
        averageVelocity = sum / FILTER_SIZE;
        
        motorSpeed += output;

        /* EMIEL CODE HERE */
        // mySetPoint = 10.;
        // averageVelocity = sum / FILTER_SIZE;
        // myNewError = mySetPoint - averageVelocity;
        // // There is no way the error can be bigger than 25 degrees per second. That has to be an error, so we set it to 0.
        // // This should basically never happen, but it might, for example with that weird first calculation.
        // if (25. < fabs(myNewError)) {
        //     myNewError = 0.;
        // }
        // myIntegral += myNewError;
        // myDerivative = myNewError - myError;
        
        // myMotorSpeed += myNewError * 6. + myIntegral * 1.;// + myDerivative * 0;
        // myMotorSpeed = constrain(myMotorSpeed, MIN_SPEED, MAX_SPEED);
        // myError = myNewError;


        // control motor
        motorSpeed = constrain(motorSpeed, MIN_SPEED, MAX_SPEED);
        // uint32_t finalMotorSpeed = (uint32_t) motorSpeed;
        // myFinalMotorSpeed = 31400;
        turn_steps_per_second((uint32_t) motorSpeed, 0);  

        // Print data
        send_data_to_pc_f("Time: %6lu, dt: %lu, CEV: %d, RV: %6.2f, AV: %6.2f, E: %6.2f, I: %6.2f, D: %6.2f, MS: %6.2f",
            now,
            dt,
            current_encoder_value,
            raw_velocity,
            averageVelocity,
            inputError,
            PIDintegral,
            PIDderivative,
            motorSpeed
        );
    }
}


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
    setpoint = 12.5;  // Gewenste snelheid in stappen per seconde
    send_text_to_pc_f("Setup completed after %d ms!", millis());
    Serial.println("Setup completed");
    delay(500);
    timer.attach(&timer_test, std::chrono::milliseconds(20));
}

// continuous loop
void loop() {

    if(timerInterruption){
        PID_control();
        timerInterruption = false;
    }

    if(emergency_stop){
        disable_motor();
        return;
    } 

    if (!encoder.begin()){
        disable_motor();
        emergency_stop = true;
        Serial.println("Encoder failure!");
        send_text_to_pc("Encoder failure!");
        return;
    }
}



    // return;
    /*
    unsigned long timestamp_next_loop = 0;
    unsigned long timestamp_previous_loop = 0;

    if (BLUETOOTH) {
        timestamp_previous_loop = micros();
        timestamp_next_loop = timestamp_previous_loop + LOOP_INTERVAL;

        // Set this once, so that the initial delta_time will be correct. If we don't do this, this value will be 0 and that ruins the first calculation.
        last_time_encoder_measured = micros();

        while (!emergency_stop) {
            // unsigned long now = micros();
            // if (timestamp_next_loop <= now) {
            //     loop_counter++;
            //     int8_t n_loops = 0;
            //     while (timestamp_next_loop <= now) {
            //         n_loops++;
            //         timestamp_next_loop += LOOP_INTERVAL;
            //     }
            //     looooops_skipped += n_loops - 1;
                


                // dt_remainder = dt_remainder + (now - timestamp_previous_loop) - LOOP_INTERVAL;

                // send_data_to_pc_f("n %u | dt %u | skipped %d | remainder %d", loop_counter, now-timestamp_previous_loop, looooops_skipped, dt_remainder);
                // timestamp_previous_loop = now;


        } // while !emergency_stop
    }     // if (bluetooth)
    else {
        Serial.println("NOT BLUETOOTH CONNECTED!");
    }
    */
// } // loop()