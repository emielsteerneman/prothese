#include "utils.h"
#include "bluetooth.h"

INTERRUPT timer;
LEFTORRIGHT LorR;

float convertToRadians(float value_in_degrees){
    return value_in_degrees * (M_PI / 180.0);
}

void timerInterrupt(){
    if (BLUETOOTH) {
        timer.interrupt = true;
    }
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
            LorR.is_left_prosthetic = true;
            sendTextToPc("User has chosen Left!");
            
            float left_transformation_matrix[3][3] = {
                {0, 0, -1}, // X flips, Z-axis effect applied
                {0, -1, 0}, // Y flips, Z-axis effect applied
                {-1, 0, 0}  // Z remains the same
            };
            // Copy left matrix into transformationMatrix
            memcpy(LorR.transformation_matrix, left_transformation_matrix, sizeof(left_transformation_matrix));
            break;
        }  
        else if (received == "R") {
            LorR.is_left_prosthetic = false;
            sendTextToPc("User has chosen Right!");

            float right_transformation_matrix[3][3] = {
                {0, 0, 1},
                {0, 1, 0},
                {-1, 0, 0}};
            // Copy right matrix into transformationMatrix
            memcpy(LorR.transformation_matrix, right_transformation_matrix, sizeof(right_transformation_matrix));
            break;
        }
        else{
            // Neither L nor R... keep waiting for valid input
        }
    }
}