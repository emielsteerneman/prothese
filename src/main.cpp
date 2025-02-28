// libraries
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Serial.h>
#include <Wire.h>
#include <AS5600.h>
#include <MahonyAHRS.h>

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

// global variables 
const uint32_t LOOP_INTERVAL = 20; // in ms
const int numReadings = 150;
const int numRounds = 40;    // Number of rounds to store quaternion values
const float MAX_SPEED = 17900.0;
const float ACCELERATION = 50000.0;//50000.0; //100
const float ERROR_MARGIN_ANGLE = 0.5;
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

    Serial.println("Calibrating of IMU complete!");
    send_text_to_pc("Calibrating of IMU complete!");
    
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

void transform_acc_data(float& ax, float& ay, float& az){
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

void transform_gyr_data(float& gx, float& gy, float& gz){
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

void algorithm1(float gx, float current_arm_angle){ // moet gx niet een pointer worden?

    if (gx > 200){ 
        gxTriggered = true;  // Set the flag to true
        gxTriggerTime = millis(); 
    }

    if (gxTriggered){
        if (current_arm_angle > 10) {
            target_arm_angle = 10;
        }

        if ((millis() - gxTriggerTime > gxCooldownPeriod) && mahony.getRoll() < -20 && current_arm_angle < 60){
            target_arm_angle = current_arm_angle;
            gxTriggered = false;
            rollTriggerTime = millis(); 
        }
    } else { //!gxTriggered
        if ((millis() - rollTriggerTime > rollCooldownPeriod) &&mahony.getRoll() < -20 && current_arm_angle < 60 /* degrees */) {
            target_arm_angle += 0.5;
        }else{
            target_arm_angle = current_arm_angle;
        }
    }

    target_arm_angle = constrain(target_arm_angle, 5, 85);

    float angle_error = target_arm_angle - current_arm_angle;
    bool target_reached = fabs(angle_error) < ERROR_MARGIN_ANGLE;

    if (target_reached){
        gxTriggered = false;
        disable_motor();
        // stepper.stop();
        digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    } else {
        digitalWrite(MOTOR_ENABLE_PIN, LOW);
        //stepper.move(angle_error * STEPS_PER_DEGREE);
        bool direction;
        if (current_arm_angle < target_arm_angle){
            direction = 0;  // FLEX
        }
        else {
            direction = 1; // EXTEND
        }
        turn_steps_per_second(20000, direction);
    }

    if (current_arm_angle < 6 || 80 < current_arm_angle) {
        disable_motor();
        //stepper.stop();
        digitalWrite(MOTOR_ENABLE_PIN, HIGH);
        emergency_stop = true;
    }
    
}

void algorithm2(float gx, float ax,  float current_arm_angle) {
    // na bereiken van roll threshold --> als gx de threshold overschrijft geldt het mapping syteem
    // zodra gx onder de threshold, motor stop.
    // als gx langer dan 1 seconde onder de threshol, motor/algoritme begint pas weer als roll threshold bereikt is
    // zodra gx onder threshold, motor stopt. Binnen een seconde weer beweging? --> algoritme wordt doorgezet
    
    // Define speed scaling factors
    const float MIN_STEPS = 10000;   // Minimum motor speed
    const float MAX_STEPS = 20000;  // Maximum motor speed
    const float GYRO_THRESHOLD = 15;  // Minimum gx value to activate movement

    // Determine speed based on gx magnitude
    float motorSteps = map(abs(gx), 0, 40, MIN_STEPS, MAX_STEPS);  
    motorSteps = constrain(motorSteps, MIN_STEPS, MAX_STEPS);  


    bool person_is_moving_arm = GYRO_THRESHOLD < abs(gx);

    if(person_is_moving_arm && gxEnabled){
        gxTriggerTime = millis(); // Store the latest time the person moved his arm
        turn_steps_per_second(motorSteps, gx < 0);
    }

    if(!person_is_moving_arm){
        disable_motor();
    }
    
    bool arm_has_not_moved_for_one_second = gxTriggerTime + gxCooldownPeriod < millis();
    gxEnabled = !arm_has_not_moved_for_one_second;
    
    if(20 < mahony.getRoll()){
        gxEnabled = true;
    }


    if (current_arm_angle < 10 || 80 < current_arm_angle) {
        disable_motor();
        emergency_stop = true;
    }

}


void algorithm3(){
    /* Algoritme 3
    Het idee is om de bovenarm alvast in de gewenste positie te brengen vooor de reiktaak en dat de onderarm daarna ingesteld kan worden.
    Dit zou bijvoorbeeld kunnen door een snelle op en neer bewegen van de bovenarm. eventuel een neer-op bewegen voor de andere kant op.
    er moet dan nog uitgezocht worden hoe de beweging gestop kan worden
    */
    const float omegaXExtendThreshold = 200;
    const float omegaXFlexThreshold = -200;

    if (mahony.getOmegaX() > omegaXExtendThreshold){
        turn_steps_per_second(20000,0);
        omegaXExtendTriggerTime = millis();
        // gxExtendTriggered = true;
    }

    if(mahony.getOmegaX() > omegaXExtendThreshold && omegaXExtendTriggerTime + gxCooldownPeriod < millis()){ //&& gxExtendTriggered
        disable_motor();
        //gxExtendTriggered = false;
    }

    if (mahony.getOmegaX() > omegaXFlexThreshold){
        turn_steps_per_second(20000,1);
        omegaXFlexTriggerTime = millis();
    }

    if(mahony.getOmegaX() > omegaXFlexThreshold && omegaXFlexTriggerTime + gxCooldownPeriod < millis()){ //&& gxExtendTriggered
        disable_motor();
    }

}

void test_run(int motor_run_counter){
    if (motor_run_counter < 50) {  
        turn_steps_per_second(31200, 0);  // Run motor forward for 1 second
        motor_run_counter++;
    } 
    else if(motor_run_counter > 49 && motor_run_counter < 100){
        turn_steps_per_second(0, 0);  // Stop motor for 1 second
        motor_run_counter++;
    }
    else if (motor_run_counter > 99 && motor_run_counter < 150) {
        turn_steps_per_second(31400, 1);  // Run motor in reverse for 1 second
        motor_run_counter++;
    } 
    else if (motor_run_counter > 149 && motor_run_counter < 200) {
        turn_steps_per_second(0, 1);  // Run motor in reverse for 1 second
        motor_run_counter++;
    } 
    else {
        motor_run_counter = 0;  // Reset counter to restart cycle
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
    delay(500); 
    setup_bluetooth();
    delay(500); 
    connect_bluetooth_to_pc();
    
    Serial.println("Waiting for user to give 'L' or 'R'");
    delay(100); 
    wait_for_user_to_give_L_R();
    delay(500); 
    setup_encoder();
    delay(500); 
    setup_imu();
    delay(500); 
    setup_motor_control();
    delay(500); 
    send_text_to_pc_f("Setup completed after %d ms!", millis());

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
    // float q0, q1, q2, q3;
    uint32_t loop_counter = 0;

    if (BLUETOOTH) {
        timestamp_next_loop = millis() + LOOP_INTERVAL;

        while (!emergency_stop) {
            if (timestamp_next_loop <= millis()) {
                while (timestamp_next_loop <= millis()) {
                    timestamp_next_loop += LOOP_INTERVAL;
                }
                
                // if (!encoder.begin()){
                //     disable_motor();
                //     emergency_stop = true;
                //     Serial.println("Encoder failure!");
                //     send_text_to_pc("Encoder failure!");
                //     break;
                // }

                // /* EMERGENCY BREAK */
                // current_encoder_value = encoder.readAngle();
                // current_arm_angle = encoder_to_arm_angle(current_encoder_value);
                // if (current_arm_angle < 10 || 80 < current_arm_angle) {
                //     disable_motor();
                //     emergency_stop = true;
                //     break;
                // }

                // while(!BLUETOOTH.connected()){
                //     // do nothing. keep checking.
                // }; 

                // Read the IMU data
                IMU.readAcceleration(acc[0], acc[1], acc[2]);
                IMU.readGyroscope(gyr[0], gyr[1], gyr[2]);
                // Transform the IMU data
                transform_acc_data(acc[0], acc[1], acc[2]);
                transform_gyr_data(gyr[0], gyr[1], gyr[2]);
                // Update the Mahony filter
                mahony.updateIMU(gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2]);
                // Get the quaternion values
                //mahony.getQuaternion(q0, q1, q2, q3); // gaat dit wel goed zo? of kan ik beter een float maken van updateIMU?
                // Drift prevention
                //drift_prevention(q0, q1, q2, q3, acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]);


                // algorithm1(gyr[0], current_arm_angle);
                // algorithm2(gyr[0], acc[1], current_arm_angle);
                // test_run(motor_run_counter);
              
                

                float rollMahony = mahony.getRoll(); // Fetch roll value
                
                send_data_to_pc_f("L %d |E %.2f |A %+.2f %+.2f %+.2f |G %+.3f %+.3f %+.3f |C %d => %.3f° |R %.3f | P %d",
                /* L */ loop_counter, 
                /* E */ angle_error,
                /* A */ acc[0], acc[1], acc[2],
                /* G */ gyr[0], gyr[1], gyr[2],
                /* C */ current_encoder_value, current_arm_angle,
                /* R */ rollMahony,
                /* P */ emergency_stop);
            }

        } // while !emergency_stop
}     // if (bluetooth)
    else {
        Serial.println("NOT BLUETOOTH CONNECTED!");
    }
} // loop()