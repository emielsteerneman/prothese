// VRAGEN:  - waar moeten welke variabelen geïniteerd worden? in of buiten loop?
//          - Hoe kan ik Mahony toevoegen? libraries downloaden geeft issues?
//          - wat doet memcpy ook alweer?

// libraries
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Serial.h>
#include <Wire.h>
#include <AS5600.h>
#include <AccelStepper.h>
#include <MahonyAHRS.h>


#include "stdint.h"
#include "IMUCalibration.h"
#include "bluetooth_stuff.h"

// define pins
#define MOTOR_STEP_PIN 3
#define MOTOR_DIR_PIN 4
#define MOTOR_ENABLE_PIN 5

// motor object
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN); 

// encoder object
AS5600 encoder;

// Mahony object
Mahony mahony;

// global variables 
const uint32_t LOOP_INTERVAL = 100; // in ms
const int numReadings = 150;
const int numRounds = 40;    // Number of rounds to store quaternion values
const float MAX_SPEED = 17900.0;
const float ACCELERATION = 50000.0;//50000.0; //100
const float ERROR_MARGIN_ANGLE = 2.0;
const float STEPS_PER_DEGREE = 10666.67;
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




void setup_imu() {
    Serial.println("Beginning IMU!");
    send_text_to_pc("Beginning IMU!");

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        send_text_to_pc("Failed to initialize IMU!");
        while (1);
    }

    Serial.println("Calibrating IMU...");
    send_text_to_pc("Calibrating IMU...");

    delay(1000);
    calibrateIMU(numReadings, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset);
    
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

void setup_motor(){
    Serial.println("Beginning Motor!");
    send_text_to_pc("Beginning Motor!");

    stepper.setPinsInverted(false, true);
    stepper.setMaxSpeed(MAX_SPEED); 
    stepper.setAcceleration(ACCELERATION);

    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);

    Serial.println("Motor initialized!");
    send_text_to_pc("Motor initialized!");
}

void transform_acc_data(float ax, float ay, float az){
    ax -= ax_offset + 1; // include gravitational constant
    ay -= ay_offset;
    az -= az_offset;

    float axTransformed = transformationMatrix[0][0] * ax + transformationMatrix[0][1] * ay + transformationMatrix[0][2] * az;
    float ayTransformed = transformationMatrix[1][0] * ax + transformationMatrix[1][1] * ay + transformationMatrix[1][2] * az;
    float azTransformed = transformationMatrix[2][0] * ax + transformationMatrix[2][1] * ay + transformationMatrix[2][2] * az;

    ax = axTransformed;
    ay = ayTransformed;
    az = azTransformed;
}

void transform_gyr_data(float gx, float gy, float gz){
    gx -= gx_offset; // / GYRO_SENSITIVITY;
    gy -= gy_offset; // / GYRO_SENSITIVITY;
    gz -= gz_offset; // / GYRO_SENSITIVITY;

    float gxTransformed = transformationMatrix[0][0] * gx + transformationMatrix[0][1] * gy + transformationMatrix[0][2] * gz;
    float gyTransformed = transformationMatrix[1][0] * gx + transformationMatrix[1][1] * gy + transformationMatrix[1][2] * gz;
    float gzTransformed = transformationMatrix[2][0] * gx + transformationMatrix[2][1] * gy + transformationMatrix[2][2] * gz;

    gx = gxTransformed;
    gy = gyTransformed;
    gz = gzTransformed;
}

void drift_prevention(float q0, float q1, float q2, float q3, float ax, float ay, float az, float gx, float gy, float gz){
    if ((q0 == q0Old) && (q1 == q1Old) && (q2 == q2Old))
    {
    q3 = q3Old;
    }
    else
    {
      // Check if az is approximately 1g and reset quaternion if so
      if (fabs(az - 1.0f) < eta)
      {        
        //  Store the quaternion values in the array
        accValues[roundCount][0] = ax;
        accValues[roundCount][1] = ay;
        accValues[roundCount][2] = az;
  
        gyrValues[roundCount][0] = gx;
        gyrValues[roundCount][1] = gy;
        gyrValues[roundCount][2] = gz;
        roundCount++;
        if (roundCount >= numRounds)
        {
          // Calculate the average of the stored quaternion values
          float avg_ax = 0, avg_ay = 0, avg_az = 0;
          float avg_gx = 0, avg_gy = 0, avg_gz = 0;
          for (int i = 0; i < numRounds; i++)
          {
            avg_ax += accValues[i][0];
            avg_ay += accValues[i][1];
            avg_az += accValues[i][2];
  
            avg_gx += gyrValues[i][0];
            avg_gy += gyrValues[i][1];
            avg_gz += gyrValues[i][2];
          }
          avg_ax /= numRounds;
          avg_ay /= numRounds;
          avg_az /= numRounds;
  
          avg_gx /= numRounds;
          avg_gy /= numRounds;
          avg_gz /= numRounds;
  
          // Check for stability within a certain range (for example, within epsilon)
          stable = true; // Assume stable unless we find a discrepancy
          for (int i = 0; i < numRounds; i++)
          {
            if (fabs(accValues[i][0] - avg_ax) > epsilon ||
                fabs(accValues[i][1] - avg_ay) > epsilon ||
                fabs(accValues[i][2] - avg_az) > epsilon ||
                fabs(gyrValues[i][0] - avg_gx) > epsilon ||
                fabs(gyrValues[i][1] - avg_gy) > epsilon ||
                fabs(gyrValues[i][2] - avg_gz) > epsilon)
            {
              stable = false; // Not stable if any value deviates
              break;
            }
          }
          // If stable, use these values to adjust the offsets
          if (stable)
          {
            ax_offset = avg_ax; // Update accelerometer offsets
            ay_offset = avg_ay;
            az_offset = avg_az - 1.0f; // Since az should be close to 1g when vertical
  
            gx_offset = avg_gx; // Update gyroscope offsets
            gy_offset = avg_gy;
            gz_offset = avg_gz;
          }
  
          // Reset the round count after processing
          roundCount = 0;
        }
      }
      // check if ay is approximately 1g, set constraints if so (2 possible cases)
      else if (fabs(ay - 1.0f) < eta)
      {
        roundCount = 0;
  
        // Apply different thresholds based on left or right prosthetic
        if (!isLeftProsthetic) // Right prosthetic
        {
          if ((q0 > 0.71) && (q1 > 0.71) && (q2 < 0) && (q3 < 0))
          {
            q0 = 0.71f;
            q1 = 0.71f;
            q2 = 0.0f;
            q3 = 0.0f;
          }
          else if ((q0 < 0.5) && (q1 < 0.5) && (q2 > 0.5) && (q3 > 0.5))
          {
            q0 = 0.5f;
            q1 = 0.5f;
            q2 = 0.5f;
            q3 = 0.5f;
          }
        }
        else // Left prosthetic
        {
          if ((q0 > 0.71) && (q1 < -0.71) && (q2 > 0) && (q3 < 0))
          {
            q0 = 0.71f;
            q1 = -0.71f;
            q2 = 0.0f;
            q3 = 0.0f;
          }
          else if ((q0 < 0.5) && (q1 > -0.5) && (q2 < -0.5) && (q3 > 0.5))
          {
            q0 = 0.5f;
            q1 = -0.5f;
            q2 = -0.5f;
            q3 = 0.5f;
          }
          roundCount = 0;
        }
      }
    }
    q0Old = q0;
    q1Old = q1;
    q2Old = q2;
    q3Old = q3;
}

float encoder_to_arm_angle(uint16_t encoder_value) {
    const float ENCODER_TO_ARM_OFFSET_DEGREES = 7.03125;

    float encoder_degrees = encoder_value * AS5600_RAW_TO_DEGREES;

    return encoder_degrees - ENCODER_TO_ARM_OFFSET_DEGREES;
}

void algorithm1(float gx, float current_arm_angle){ // beter om de variabelen globaal te maken?
    float target_arm_angle = 5;
    bool gxTriggered = false;

    if (gx > 200){ // m/s2
        gxTriggered = true;  // Set the flag to true
    }

    if (gxTriggered){
        if (current_arm_angle > 5) {
            target_arm_angle = 5;
        }
    } else {
        if (mahony.getRoll() < -20 && current_arm_angle < 90 /* degrees */) {
            target_arm_angle += 0.5;
        }
    }

    float angle_error = target_arm_angle - current_arm_angle;
    bool target_reached = fabs(angle_error) < ERROR_MARGIN_ANGLE;

    if (target_reached){
        stepper.stop();
        digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    } else {
        digitalWrite(MOTOR_ENABLE_PIN, LOW);
        stepper.move(angle_error * STEPS_PER_DEGREE);
    }

    if (current_arm_angle < 4 || 91 < current_arm_angle) {
        stepper.stop();
        digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    }
}

void wait_for_user_to_give_L_R(){
    // This needs the bluetooth to be running

    send_text_to_pc("Do you have a left (L) or right (R) elbow prosthetic? Enter 'L' or 'R':");

    while(true){
        while(!BLUETOOTH.connected()){
            // do nothing. keep checking.
        }; 

        // Check if we have input from the user. If not, continue
        if( !pc_has_written() )
            continue;

        Serial.println("User has given input!");

        // We received something from the user! Read it.
        String received = get_pc_input();

        // User indicated Left
        if (received == "L") {
            isLeftProsthetic = true;
            send_text_to_pc("User has chosen Left!");
            
            float leftTransformationMatrix[3][3] = {
                {0, 0, -1}, // X flips, Z-axis effect applied
                {0, -1, 0}, // Y flips, Z-axis effect applied
                {-1, 0, 0}  // Z remains the same
            };
            // Copy left matrix into transformationMatrix
            memcpy(transformationMatrix, leftTransformationMatrix, sizeof(leftTransformationMatrix));
            break;
        }  
        else if (received == "R") {
            isLeftProsthetic = false;
            send_text_to_pc("User has chosen Right!");

            float rightTransformationMatrix[3][3] = {
                {0, 0, 1},
                {0, 1, 0},
                {-1, 0, 0}};
            // Copy right matrix into transformationMatrix
            memcpy(transformationMatrix, rightTransformationMatrix, sizeof(rightTransformationMatrix));
            break;
        }
        else{
            // Neither L nor R... keep waiting for valid input
        }
    }
}

// setup, runs once
void setup() {
    Serial.begin(115200);
    Wire.begin();

    delay(500); setup_bluetooth();
    delay(500); connect_bluetooth_to_pc();
    
    Serial.println("Waiting for user to give L or R");
    delay(100); wait_for_user_to_give_L_R();
    delay(500); setup_encoder();
    delay(500); setup_imu();
    delay(500); setup_motor();
    delay(500); send_text_to_pc_f("Setup completed after %d ms!", millis());

    mahony.begin(10);
}

// continuous loop
void loop() {
    float acc[3] = {0, 0, 0};
    float gyr[3] = {0, 0, 0};
    uint32_t timestamp_next_loop = 0;
    uint16_t current_encoder_value = 0;
    float current_arm_angle = 0.;
    float angle_error = 0;
    float q0, q1, q2, q3;
    uint32_t loop_counter = 0;

    if (BLUETOOTH) {
        timestamp_next_loop = millis() + LOOP_INTERVAL;

        while (true) {
            if (timestamp_next_loop <= millis()) {
                while (timestamp_next_loop <= millis()) {
                    timestamp_next_loop += LOOP_INTERVAL;
                }

                while(!BLUETOOTH.connected()){
                    // do nothing. keep checking.
                }; 

                // Read the IMU data
                IMU.readAcceleration(acc[0], acc[1], acc[2]);
                IMU.readGyroscope(gyr[0], gyr[1], gyr[2]);
                // Transform the IMU data
                transform_acc_data(acc[0], acc[1], acc[2]);
                transform_gyr_data(gyr[0], gyr[1], gyr[2]);
                // Update the Mahony filter
                mahony.updateIMU(gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2]);
                // Get the quaternion values
                mahony.getQuaternion(q0, q1, q2, q3);
                // Drift prevention
                drift_prevention(q0, q1, q2, q3, acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]);


                current_encoder_value = encoder.readAngle();
                current_arm_angle = encoder_to_arm_angle(current_encoder_value);

                algorithm1(gyr[0], current_arm_angle);
                float rollMahony = mahony.getRoll(); // Fetch roll value
                
                send_data_to_pc_f("L %d |E %.2f |A %+.2f %+.2f %+.2f |G %+.3f %+.3f %+.3f |C %d => %.3f° |R %.3f",
                /* L */ loop_counter, 
                /* E */ angle_error,
                /* A */ acc[0], acc[1], acc[2],
                /* G */ gyr[0], gyr[1], gyr[2],
                /* C */ current_encoder_value, current_arm_angle,
                /* R */ rollMahony);
            }

            stepper.run();
        } // while true
    } // if (bluetooth)
    else {
        Serial.println("NOT BLUETOOTH CONNECTED!");
    }
} // loop()