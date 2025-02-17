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
#include <stdarg.h>

#include "stdint.h"
#include "IMUCalibration.h"

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

// BlueTooth service and characteristics
BLEService myService("12345678-1234-5678-1234-56789abcdef0"); // service UUID
// https://docs.arduino.cc/libraries/arduinoble/#BLECharacteristic%20Class
BLECharacteristic ArduinoToPc("12345678-1234-5678-1234-56789abcdef1", BLERead | BLENotify, 256); // read and notify UUID
BLEStringCharacteristic PcToArduino("12345678-1234-5678-1234-56789abcdef2", BLEWrite, 32); // write UUID
BLEDevice bluetooth;

/* HERE IS ALL THE SEND-THINGS-TO-PC STUFF */
// data buffer
char send_buffer_string[256];

void send_text_to_pc(const char* string){
    sprintf(send_buffer_string, "TEXT%s", string);
    ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));
}

void send_text_to_pc_f(const char* format, ...){
    // Start by writing "TEXT" into the buffer
    strcpy(send_buffer_string, "TEXT");
    // Do all the other magic string stuff that places the formatted string into the buffer
    va_list args;
    va_start(args, format);
    vsnprintf(send_buffer_string + 4, sizeof(send_buffer_string) - 4, format, args);
    va_end(args);
    // Send the buffer to the PC via Bluetooth
    ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));
}
/* END OF SEND-THINGS-TO-PC STUFF */

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
    float target_arm_angle=5;
    float angle_error = 0;
    float rollMahony = 0;
    bool gxTriggered = false;

    target_arm_angle = constrain(target_arm_angle, 0, 90);

    rollMahony = mahony.getRoll(); // Fetch roll value

    if (gx > 200) // m/s2
        {
            gxTriggered = true;  // Set the flag to true
        }

    if (gxTriggered) 
        {
            if (current_arm_angle > 5) 
            {
                target_arm_angle = 5;
            } 
            else 
            {
            gxTriggered = false;
            }
        } 
    else 
        {
            if (rollMahony < -20 && current_arm_angle < 90)//degrees
            {
                target_arm_angle += 0.5;
            }
        }

    angle_error = target_arm_angle - current_arm_angle;

    bool target_reached = fabs(angle_error) < ERROR_MARGIN_ANGLE;

    if (target_reached)
    {
        stepper.stop();
        digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    }
    else
    {
        digitalWrite(MOTOR_ENABLE_PIN, LOW);
        stepper.move(angle_error * STEPS_PER_DEGREE);
    }

    if (current_arm_angle < 4 || 91 < current_arm_angle) 
    {
                stepper.stop();
        digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    }
}

void wait_for_user_to_give_L_R(){
    // This needs the bluetooth to be running

    send_text_to_pc("Do you have a left (L) or right (R) elbow prosthetic? Enter 'L' or 'R':");

    while(true){
        while(!bluetooth.connected()){
            // do nothing. keep checking.
        }; 

        // Check if we have input from the user. If not, continue
        if( !PcToArduino.written() )
            continue;

        Serial.println("User has given input!");

        // We received something from the user! Read it.
        String received = PcToArduino.value();

        // User indicated Left
        if (received == "L") {
            isLeftProsthetic = true;
            
            send_text_to_pc("User has chosen Left!");
            // sprintf(send_buffer_string, "TEXTUser has chosen Left!");
            // ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));
            
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
            
            // sprintf(send_buffer_string, "TEXTUser has chosen Right!");
            // ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));
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

    delay(1000);
    setup_bluetooth();
    delay(1000);

    /* THIS PART ESTABLISHES THE BLUETOOTH CONNECTION BETWEEN THE ARDUINO AND THE PYTHON CODE */
    Serial.println("Waiting for the PC to connect to the Arduino Bluetooth...");
    while(!bluetooth){
        bluetooth = BLE.central();
    }
    while(!bluetooth.connected());
    Serial.println("PC connected to the Arduino Bluetooth!");

    // This is needed because for some reason, the first few writes to the PC are not received.. This keeps writing
    // until the PC has received the message and responds. We then know that the PC is ready to receive messages.
    while(!PcToArduino.written()){
        delay(200);
        while(!bluetooth.connected());
        send_text_to_pc("INIT");
        Serial.print(".. Still waiting\r");
    }
    /* THE CONNECTION IS NOW MADE BETWEEN THE ARDUINO AND THE PYTHON CODE */

    Serial.println("Waiting for user to give L or R");
    delay(100); wait_for_user_to_give_L_R();
    delay(500); setup_encoder();
    delay(500); setup_imu();
    delay(500); setup_motor();
    delay(500); send_text_to_pc_f("Setup completed after %d ms!", millis());

    mahony.begin(50);
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

    if (bluetooth) {
        timestamp_next_loop = millis() + LOOP_INTERVAL;

        while (true) {
            if (timestamp_next_loop <= millis()) {
                while (timestamp_next_loop <= millis()) {
                    timestamp_next_loop += LOOP_INTERVAL;
                }

                while(!bluetooth.connected()){
                    // do nothing. keep checking.
                }; 

                IMU.readAcceleration(acc[0], acc[1], acc[2]);
                transform_acc_data(acc[0], acc[1], acc[2]);

                IMU.readGyroscope(gyr[0], gyr[1], gyr[2]);
                transform_gyr_data(gyr[0], gyr[1], gyr[2]);
                             
                mahony.updateIMU(gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2]);

                mahony.getQuaternion(q0, q1, q2, q3);

                drift_prevention(q0, q1, q2, q3, acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]);

                current_encoder_value = encoder.readAngle();
                current_arm_angle = encoder_to_arm_angle(current_encoder_value);
                current_arm_angle = constrain(current_arm_angle, 0, 90);

                algorithm1(gyr[0], current_arm_angle);
                
                sprintf(send_buffer_string, "L %d |E %.2f |A %+.2f %+.2f %+.2f |G %+.3f %+.3f %+.3f |° %d %.3f",
                loop_counter, 
                angle_error,
                acc[0], acc[1], acc[2],
                gyr[0], gyr[1], gyr[2],
                current_encoder_value, current_arm_angle);
                ArduinoToPc.writeValue(send_buffer_string, sizeof(send_buffer_string));
            }

            stepper.run();
        } // while true
    } // if (bluetooth)
    else {
        Serial.println("NOT BLUETOOTH CONNECTED!");
    }
} // loop()