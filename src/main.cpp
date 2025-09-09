// Buigen is +12.5
// Strekken is -12.5

// libraries
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Serial.h>
#include <Wire.h>
#include <MahonyAHRS.h>
#include <mbed.h>
#include <ams_as5048b.h>

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

#define ENCODER_RAW 1 
#define ENCODER_DEGREES 3 

// objects
mbed::Ticker timer;
AMS_AS5048B encoder;
Mahony mahony;

const float ENCODER_OFFSET = 66.51; // set value manually based on initial position of the arm

// variables timerInterrupt
volatile bool timer_interrupt = false;

// variables setupIMU
const int numReadings = 150; // calibration samples
float ax_offset = 0, ay_offset = 0, az_offset = 0; // initialize offset values
float gx_offset = 0, gy_offset = 0, gz_offset = 0; // initialize offset values

// variables calculateArmAngle
uint16_t encoder_value = 0;

// variables PIDControl
unsigned long previous_PID_timestamp = 0;
unsigned long previous_encoder_timestamp = 0;
float elbow_angle = 0;
float previous_elbow_angle = 0;
float previous_encoder_value = 0; 
#define FILTER_SIZE 12  // Number of values for moving average
double velocity_buffer[FILTER_SIZE] = {0};  
int velocity_index = 0;  // Index for buffer
double reference_velocity = 12.5;
double input_velocity = 0;
double output_velocity = 0; 
double K_p = 10.0, K_i = 2.0, K_d = 10.0;  // Tuning parameters
double error_velocity = 0; 
double previous_error_velocity = 0; 
double PID_integral = 0;  
double PID_derivative = 0; 
float average_velocity = 0; 
float motor_speed = 20000;
const float MAX_SPEED = 31400.0;
const float MIN_SPEED = 20000;
uint32_t log_counter = 0;

// variables transformData
float transformation_matrix[3][3];

// variables algorithm1
unsigned long omega_x_trigger_timestamp = 0; // Stores the last time gx was triggered
const unsigned long OMEGA_X_COOLDOWN_PERIOD = 500; // Cooldown period in milliseconds
unsigned long roll_trigger_timestamp = 0; // Stores the last time roll was triggered
const unsigned long ROLL_COOLDOWN_PERIOD = 1000; // Cooldown period in milliseconds
bool omega_x_triggered = false; // Stores the last time gx was triggered

// variables algorithm2
const float OMEGA_X_EXTEND_THRESHOLD = -100;
const float OMEGA_X_FLEX_THRESHOLD = 100;
bool extend_motor_running = false;
bool flex_motor_running = false;
bool extend_cooldown_passed = false;
bool flex_cooldown_passed = false;
const unsigned long EXTRA_COOLDOWN_PERIOD = 500;
unsigned long omega_x_extend_trigger_timestamp = 0;
unsigned long omega_x_flex_trigger_timestamp = 0;
unsigned long extend_stop_timestamp = 0;
unsigned long flex_stop_timestamp = 0;

// varibales waitForLRInput
bool is_left_prosthetic = false;

// variables loop
bool emergency_stop = false;
uint8_t unsafe_encoder_measurements = 0;
float acc[3] = {0, 0, 0};
float gyr[3] = {0, 0, 0};

// variables ModelPredictiveControl
const float ELBOW_RADIUS = 42.426; // mm
const float LINEAR_VELOCITY = 0.0003125; // mm/s --> lead/(microsteps per revolution) = 2/(2*16*200) = 0.0003125 mm/step
float motor_speed_MPC = 0.0; 
const float DISTANCE_AXIS_PIN = 30.0; // mm
const float DISTANCE_AXIS_MOTOR = 60.0; // mm

// Helper functions

float convertToRadians(float value_in_degrees){
    return value_in_degrees * (M_PI / 180.0);
}

void timerInterrupt(){
    if (BLUETOOTH) {
        timer_interrupt = true;
    }
}

void transformAccelerometerData(float& ax, float& ay, float& az){
    ax -= ax_offset + 1; // include gravitational constant
    ay -= ay_offset;
    az -= az_offset;

    float ax_transformed = transformation_matrix[0][0] * ax + transformation_matrix[0][1] * ay + transformation_matrix[0][2] * az;
    float ay_transformed = transformation_matrix[1][0] * ax + transformation_matrix[1][1] * ay + transformation_matrix[1][2] * az;
    float az_transformed = transformation_matrix[2][0] * ax + transformation_matrix[2][1] * ay + transformation_matrix[2][2] * az;

    ax = ax_transformed;
    ay = ay_transformed;
    az = az_transformed;
}

void transformGyroscopeData(float& gx, float& gy, float& gz){
    gx -= gx_offset; // / GYRO_SENSITIVITY;
    gy -= gy_offset; // / GYRO_SENSITIVITY;
    gz -= gz_offset; // / GYRO_SENSITIVITY;

    float gx_transformed = transformation_matrix[0][0] * gx + transformation_matrix[0][1] * gy + transformation_matrix[0][2] * gz;
    float gy_transformed = transformation_matrix[1][0] * gx + transformation_matrix[1][1] * gy + transformation_matrix[1][2] * gz;
    float gz_transformed = transformation_matrix[2][0] * gx + transformation_matrix[2][1] * gy + transformation_matrix[2][2] * gz;

    gx = gx_transformed;
    gy = gy_transformed;
    gz = gz_transformed;
}

// Main functions

void move_to_5_degrees(float omega_x){
    while(true){
        unsigned long PID_timestamp = micros();
                    
        // Read encoder
        unsigned long encoder_timestamp = micros();
        encoder_value = encoder.angleR(ENCODER_RAW, true);
        elbow_angle = (encoder.angleR(ENCODER_DEGREES, true)*-1)-ENCODER_OFFSET;
        if(elbow_angle < 0){
            elbow_angle += 360;
        }
        unsigned long delta_PID_timestamp = PID_timestamp - previous_PID_timestamp;
        previous_PID_timestamp = PID_timestamp;

        // Calculate velocity in degrees per second
        float delta_elbow_angle = elbow_angle - previous_elbow_angle;
        float delta_encoder_timestamp = (encoder_timestamp - previous_encoder_timestamp) / 1000000.0; // Convert to seconds
        float raw_velocity = delta_elbow_angle / delta_encoder_timestamp;

        previous_elbow_angle = elbow_angle;
        previous_encoder_timestamp = encoder_timestamp;

        // Store value in circular moving average buffer
        velocity_buffer[velocity_index] = raw_velocity;
        velocity_index = (velocity_index + 1) % FILTER_SIZE;

        // Compute moving average
        float velocity_sum = 0;
        for (int i = 0; i < FILTER_SIZE; i++) {
            velocity_sum += velocity_buffer[i];
        }
        average_velocity = velocity_sum / FILTER_SIZE;

        /* EMERGENCY BREAK */
        if (elbow_angle < 3 || 92 < elbow_angle) {
            unsafe_encoder_measurements++;
            if(unsafe_encoder_measurements > 3){
                sendTextToPcf("EMERGENCY STOPPED: Elbow angle out of bounds: %6.2f", elbow_angle);
                disableMotor();
                emergency_stop = true;
                return;
            }else{
                sendTextToPcf("WARNING %d: Elbow angle out of bounds: %6.2f", unsafe_encoder_measurements, elbow_angle);
            }
        }else{
            unsafe_encoder_measurements = 0;
        }

        float error = 5 - elbow_angle;

        // Stop when we reached our target angle (or at least close enough)
        if( fabs(error) < 1){
            disableMotor();
            sendTextToPc("Reached 5 degrees");
            return;
        }

        turnStepsPerSecond(31400, error < 0); // True = extend

        sendDataToPcf("N: %4d, t: %6lu, dt: %lu, ENC_DEG: %5.2f, REF: %5.2f, VEL_RAW: %5.2f, VEL_AVG: %5.2f, I: %5.2f, D: %5.2f, VEL_OUT: %5.2f, MS_MPC: %5.2f, ax: %5.2f, ay: %5.2f, az: %5.2f, gx: %5.2f, gy: %5.2f, gz: %5.2f, OX: %5.2f",
            log_counter,
            PID_timestamp,
            delta_PID_timestamp,
            elbow_angle,
            input_velocity,
            raw_velocity,
            average_velocity,
            PID_integral,
            PID_derivative,
            output_velocity,
            motor_speed_MPC,
            acc[0],
            acc[1],
            acc[2],
            gyr[0],
            gyr[1],
            gyr[2],
            omega_x
        );
        log_counter++;

    }
}

void algorithm1(float& omega_x, float elbow_angle, float gx){ // moet gx niet een pointer worden?
    /* Algorithm 1
    When the arm is brought to roll > 20 degrees, the elbow angle will increase until the users removes it from this position.
    The arm can then move freely until gx is triggered or until it is back in this > 20 degrees position. 
    When gx is triggered by a fast short movement downwards of the arm, the arm angle reduces until it is in the > position.
    If it stays in this position for longer than one second, the arm angle will increase again.
    A cooldownperiod of 0.5 seconds has been build in, to prevent the triggering of the roll right after gx has been triggered. */

    if (gx < -100 && !omega_x_triggered) { // Check if gx is triggered
        omega_x_triggered = true;  // Set the flag to true
        omega_x_trigger_timestamp = millis(); 
    }

    if (omega_x_triggered){
        if (elbow_angle > 5) {
            input_velocity = -fabs(reference_velocity);
        }

        if ((millis() - omega_x_trigger_timestamp > OMEGA_X_COOLDOWN_PERIOD) && mahony.getRoll() > 20 ){
            input_velocity = 0;
            omega_x_triggered = false;
            roll_trigger_timestamp = millis(); 
        }
    } else { //!gxTriggered
        if ((millis() - roll_trigger_timestamp > ROLL_COOLDOWN_PERIOD) &&mahony.getRoll() > 20 && elbow_angle < 88) {
            input_velocity = fabs(reference_velocity);
        }else{
            input_velocity = 0;
        }
    } 

     // ANGLE-BASED CONDITIONS
     if (elbow_angle <= 5) {
        if(input_velocity < 0){
            input_velocity = 0;
        }
    }

    if (elbow_angle >= 88) {
        if(input_velocity > 0){
            input_velocity = 0;
        }
    }
    
}

void algorithm2(float omega_x, float elbow_angle, float gx){
    /* Algorithm 2
    The idea is to first bring the upper arm into the desired position for the reaching task, after which the forearm can be adjusted.
    This could, for example, be done by a quick up-and-down movement of the upper arm—possibly a down-up movement for the opposite direction.
    */

    // EXTEND MOVEMENT
    if ((gx < OMEGA_X_EXTEND_THRESHOLD) && !extend_motor_running) {    
        // Ensure extra cooldown has passed before starting again
        if (millis() - extend_stop_timestamp > EXTRA_COOLDOWN_PERIOD) {
            input_velocity = -fabs(reference_velocity);
            omega_x_extend_trigger_timestamp = millis();
            extend_motor_running = true;  
            extend_cooldown_passed = false;        }
    }

    // Check if cooldown has passed
    if (extend_motor_running && (millis() - omega_x_extend_trigger_timestamp > OMEGA_X_COOLDOWN_PERIOD)) {
        extend_cooldown_passed = true;
    }

    // Stop motor only if cooldown has passed AND gx is triggered again
    if (extend_cooldown_passed && gx < OMEGA_X_EXTEND_THRESHOLD) {
        input_velocity = 0;
        extend_motor_running = false;
        extend_cooldown_passed = false;
        extend_stop_timestamp = millis(); // Store stop time to enforce extra cooldown
    }

    // FLEX MOVEMENT
    if (gx > OMEGA_X_FLEX_THRESHOLD && !flex_motor_running) {  
        // Ensure extra cooldown has passed before starting again
        if (millis() - flex_stop_timestamp > EXTRA_COOLDOWN_PERIOD) {
            input_velocity = fabs(reference_velocity);
            omega_x_flex_trigger_timestamp = millis();
            flex_motor_running = true;  
            flex_cooldown_passed = false;
        }
    }

    // Check if cooldown has passed
    if ( (millis() - omega_x_flex_trigger_timestamp > OMEGA_X_COOLDOWN_PERIOD) && flex_motor_running ) { // 
        flex_cooldown_passed = true;
    }

    // Stop motor only if cooldown has passed AND gx is triggered again
    if (flex_cooldown_passed && gx > OMEGA_X_FLEX_THRESHOLD) {
        input_velocity = 0;
        flex_motor_running = false;
        flex_cooldown_passed = false;
        flex_stop_timestamp = millis(); // Store stop time to enforce extra cooldown
    }

     // ANGLE-BASED CONDITIONS
     if (elbow_angle <= 5) {
        if(input_velocity < 0){
            input_velocity = 0;
        }
    }

    if (elbow_angle >= 88) {
        if(input_velocity > 0){
            input_velocity = 0;
        }
    }
}

void setupIMU() {
    Serial.println("Beginning IMU!");
    sendTextToPc("Beginning IMU!");

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        sendTextToPc("Failed to initialize IMU!");
        while(1);
    }

    Serial.println("Calibrating IMU...");
    sendTextToPc("Calibrating IMU...");

    delay(1000);
    calibrateIMU(numReadings, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset);

    Serial.println("Calibrating of IMU complete!");
    sendTextToPc("Calibrating of IMU complete!");
    
}

void setupEncoder() {
    Serial.println("Beginning Encoder!");
    sendTextToPc("Beginning Encoder!");
    encoder.begin();
    Serial.println("Encoder initialized!");
    sendTextToPc("Encoder initialized!");
}

void waitForLRInput(){
    // This needs the bluetooth to be running

    sendTextToPc("Do you have a left (L) or right (R) elbow prosthetic? Enter 'L' or 'R':");

    while(true){
        while(!BLUETOOTH.connected()){
            // do nothing. keep checking.
        }; 

        // Check if we have input from the user. If not, continue
        if( !pcHasWritten())
            continue;

        Serial.println("User has given input!");

        // We received something from the user! Read it.
        String received = getPcInput();

        // User indicated Left
        if (received == "L") {
            is_left_prosthetic = true;
            sendTextToPc("User has chosen Left!");
            
            float left_transformation_matrix[3][3] = {
                {0, 0, -1}, // X flips, Z-axis effect applied
                {0, -1, 0}, // Y flips, Z-axis effect applied
                {-1, 0, 0}  // Z remains the same
            };
            // Copy left matrix into transformationMatrix
            memcpy(transformation_matrix, left_transformation_matrix, sizeof(left_transformation_matrix));
            break;
        }  
        else if (received == "R") {
            is_left_prosthetic = false;
            sendTextToPc("User has chosen Right!");

            float right_transformation_matrix[3][3] = {
                {0, 0, 1},
                {0, 1, 0},
                {-1, 0, 0}};
            // Copy right matrix into transformationMatrix
            memcpy(transformation_matrix, right_transformation_matrix, sizeof(right_transformation_matrix));
            break;
        }
        else{
            // Neither L nor R... keep waiting for valid input
        }
    }
}

void MPCWithPIDControl(float omega_x){
    if (BLUETOOTH) {
        // Determine time since last tick
        unsigned long PID_timestamp = micros();
                    
        // Read encoder
        unsigned long encoder_timestamp = micros();
        encoder_value = encoder.angleR(ENCODER_RAW, true);
        elbow_angle = (encoder.angleR(ENCODER_DEGREES, true)*-1)-ENCODER_OFFSET;

        previous_encoder_value = encoder_value;
        if(elbow_angle < 0){
            elbow_angle += 360;
        }

        /* EMERGENCY BREAK */
        if (elbow_angle < 3 || 93 < elbow_angle) {
            unsafe_encoder_measurements++;
            if(unsafe_encoder_measurements > 3){
                sendTextToPcf("EMERGENCY STOPPED: Elbow angle out of bounds: %6.2f", elbow_angle);
                disableMotor();
                emergency_stop = true;
                return;
            }else{
                sendTextToPcf("WARNING %d: Elbow angle out of bounds: %6.2f", unsafe_encoder_measurements, elbow_angle);
            }
        }else{
            unsafe_encoder_measurements = 0;
        }

        // Ignore any measurements that are not possible
        if (elbow_angle > 95 && elbow_angle < 353) {
            sendTextToPcf("IMPOSSIBLE ANGLE: %6.2f", elbow_angle);
            return;
        }
        
        unsigned long delta_PID_timestamp = PID_timestamp - previous_PID_timestamp;
        previous_PID_timestamp = PID_timestamp;

        // Calculate velocity in degrees per second
        float delta_elbow_angle = elbow_angle - previous_elbow_angle;
        float delta_encoder_timestamp = (encoder_timestamp - previous_encoder_timestamp) / 1000000.0; // Convert to seconds
        float raw_velocity = delta_elbow_angle / delta_encoder_timestamp;

        previous_elbow_angle = elbow_angle; // update arm angle
        previous_encoder_timestamp = encoder_timestamp; // update time

        // Store value in circular moving average buffer
        velocity_buffer[velocity_index] = raw_velocity;
        velocity_index = (velocity_index + 1) % FILTER_SIZE;

        // Compute moving average
        float velocity_sum = 0;
        for (int i = 0; i < FILTER_SIZE; i++) {
            velocity_sum += velocity_buffer[i];
        }
        average_velocity = velocity_sum / FILTER_SIZE;

        // PID-calculations           
        error_velocity = fabs(input_velocity) - fabs(average_velocity);  
        PID_integral += error_velocity;  
        PID_derivative = error_velocity - previous_error_velocity;  
        output_velocity = K_p * error_velocity + K_i * PID_integral + K_d * PID_derivative;  
        previous_error_velocity = error_velocity; 

        float theta = convertToRadians(elbow_angle+45);
        float sin_theta = sin(theta);
        float cos_theta = cos(theta);

        float numerator = fabs(
            (DISTANCE_AXIS_PIN - ELBOW_RADIUS * sin_theta) * (ELBOW_RADIUS * cos_theta) +
            ELBOW_RADIUS * sin_theta * (DISTANCE_AXIS_MOTOR + ELBOW_RADIUS * cos_theta)
        );

        float denominator = sqrt(
            pow((DISTANCE_AXIS_PIN - ELBOW_RADIUS * sin_theta), 2) +
            pow((DISTANCE_AXIS_MOTOR + ELBOW_RADIUS * cos_theta), 2)
        );

        float distance = numerator / denominator;

        motor_speed_MPC = ((convertToRadians(fabs(reference_velocity)) * distance) / LINEAR_VELOCITY);

        if (input_velocity !=0){
            motor_speed = motor_speed_MPC + output_velocity;
            motor_speed = constrain(motor_speed, MIN_SPEED, MAX_SPEED);
            turnStepsPerSecond((uint32_t) motor_speed, input_velocity < 0);  
        }
        else{
            motor_speed = 0;
            disableMotor();
        }

        sendDataToPcf("N: %4d, t: %6lu, dt: %lu, ENC_DEG: %5.2f, REF: %5.2f, VEL_RAW: %5.2f, VEL_AVG: %5.2f, I: %5.2f, D: %5.2f, VEL_OUT: %5.2f, MS_MPC: %5.2f, ax: %5.2f, ay: %5.2f, az: %5.2f, gx: %5.2f, gy: %5.2f, gz: %5.2f, OX: %5.2f",
            log_counter,
            PID_timestamp,
            delta_PID_timestamp,
            elbow_angle,
            input_velocity,
            raw_velocity,
            average_velocity,
            PID_integral,
            PID_derivative,
            output_velocity,
            motor_speed_MPC,
            acc[0],
            acc[1],
            acc[2],
            gyr[0],
            gyr[1],
            gyr[2],
            omega_x
        );
        log_counter++;
    }
}

// setup, runs once
void setup() {
    Serial.begin(115200);
    Wire.begin(); delay(500); 
    setupBluetooth(); delay(500); 
    connectBluetoothToPc(); delay(500);
    Serial.println("Waiting for user to give 'L' or 'R'");
    waitForLRInput(); delay(500);
    setupEncoder(); delay(500); 
    setupIMU(); delay(500);
    setupMotorControl(); delay(500); 
    Serial.println("Setup completed");
    sendTextToPcf("Setup completed after %d ms!", millis()); 
    delay(500);

    timer.attach(&timerInterrupt, std::chrono::milliseconds(20));

    mahony.begin(50);
}

// continuous loop
void loop() {

    if(emergency_stop){
        while(1){
            disableMotor();
            sendTextToPc("EMERGENCY STOPPED");
            delay(1000);
        }
    } 

    // Read the IMU data
    IMU.readAcceleration(acc[0], acc[1], acc[2]);
    IMU.readGyroscope(gyr[0], gyr[1], gyr[2]);

    // Transform the IMU data
    transformAccelerometerData(acc[0], acc[1], acc[2]);
    transformGyroscopeData(gyr[0], gyr[1], gyr[2]);

    float omega_x = 0;
    float omega_y = 0;
    float omega_z = 0;

    mahony.updateIMU(gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2], omega_x, omega_y, omega_z);

    if(timer_interrupt){
        noInterrupts();
        algorithm1(omega_x, elbow_angle, gyr[0]);
        // algorithm2(omega_x, elbow_angle, gyr[0]);
        MPCWithPIDControl(omega_x);
        timer_interrupt = false;
        interrupts();
    }

    if (pcHasWritten()){
        if (getPcInput() == "q") {
            sendTextToPc("STOPPED BY USER");
            disableMotor();
            emergency_stop = true;
        } else if (getPcInput() == "m") {
            sendTextToPc("MOVING TO 5 DEGREES");
            move_to_5_degrees(omega_x);
        } else if (getPcInput() == "QUIT") {
            sendTextToPc("STOPPED BY PYTHON");
            disableMotor();
            emergency_stop = true;
        }
    }
}