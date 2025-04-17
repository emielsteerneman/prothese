#include "algorithms.h"

void algorithm1(float omega_x, float elbow_angle){
     /* Algorithm 1
    When the arm is brought to roll > 20 degrees, the elbow angle will increase until the users removes it from this position.
    The arm can then move freely until gx is triggered or until it is back in this > 20 degrees position. 
    When gx is triggered by a fast short movement downwards of the arm, the arm angle reduces until it is in the > position.
    If it stays in this position for longer than one second, the arm angle will increase again.
    A cooldownperiod of 0.5 seconds has been build in, to prevent the triggering of the roll right after gx has been triggered. */

    if (omega_x < -7 && !algorithm1.omega_x_triggered) { // Check if gx is triggered
        omega_x_triggered = true;  // Set the flag to true
        omega_x_trigger_timestamp = millis(); 
    }

    if (omega_x_triggered){
        if (elbow_angle > 5) {
            // motor_direction = 1; //target_arm_angle = 5;
            input_velocity = -fabs(reference_velocity);
            // turnStepsPerSecond(motor_speed, motor_direction);
        }

        if ((millis() - omega_x_trigger_timestamp > OMEGA_X_COOLDOWN_PERIOD) && mahony.getRoll() > 20 ){ //&& current_arm_angle < 85
            input_velocity = 0;
            // disableMotor();//target_arm_angle = current_arm_angle;
            omega_x_triggered = false;
            roll_trigger_timestamp = millis(); 
        }
    } else { //!gxTriggered
        if ((millis() - roll_trigger_timestamp > ROLL_COOLDOWN_PERIOD) &&mahony.getRoll() > 20 && elbow_angle < 88) { //&& current_arm_angle < 85 /* degrees */
            // motor_direction = 0; //target_arm_angle += 0.5;
            input_velocity = fabs(reference_velocity);
            // turnStepsPerSecond(motor_speed, motor_direction);

        }else{
            // disableMotor();//target_arm_angle = current_arm_angle;
            input_velocity = 0;
        }
    } 

     // ANGLE-BASED CONDITIONS
     if (elbow_angle <= 5) {
        if(input_velocity < 0){//motor_direction == 1
            // disableMotor();
            input_velocity = 0;
        }
    }

    if (elbow_angle >= 88) {
        if(input_velocity > 0){ //motor_direction == 0
            // disableMotor();
            input_velocity = 0;
        }
    }
    
}

void algorithm2(float omega_x, float elbow_angle){
    /* Algoritme 2
    Het idee is om de bovenarm alvast in de gewenste positie te brengen vooor de reiktaak en dat de onderarm daarna ingesteld kan worden.
    Dit zou bijvoorbeeld kunnen door een snelle op en neer bewegen van de bovenarm. eventuel een neer-op bewegen voor de andere kant op.
    er moet dan nog uitgezocht worden hoe de beweging gestop kan worden
    */

    // EXTEND MOVEMENT
    if ((omega_x < OMEGA_X_EXTEND_THRESHOLD) && !extend_motor_running) {    
        // Ensure extra cooldown has passed before starting again
        if (millis() - extend_stop_timestamp > EXTRA_COOLDOWN_PERIOD) {
            // motor_direction = 0;
            input_velocity = fabs(reference_velocity);
            // turnStepsPerSecond(motor_speed, motor_direction);
            omega_x_extend_trigger_timestamp = millis();
            extend_motor_running = true;  
            extend_cooldown_passed = false;        }
    }

    // Check if cooldown has passed
    if (extend_motor_running && (millis() - omega_x_extend_trigger_timestamp > OMEGA_X_COOLDOWN_PERIOD)) { //
        extend_cooldown_passed = true;
    }

    // Stop motor only if cooldown has passed AND gx is triggered again
    if (extend_cooldown_passed && omega_x < OMEGA_X_EXTEND_THRESHOLD) {
        // disableMotor();
        input_velocity = 0;
        extend_motor_running = false;
        extend_cooldown_passed = false;
        extend_stop_timestamp = millis(); // Store stop time to enforce extra cooldown
    }

    // FLEX MOVEMENT
    if (omega_x > OMEGA_X_FLEX_THRESHOLD && !flex_motor_running) {  
        // Ensure extra cooldown has passed before starting again
        if (millis() - flex_stop_timestamp > EXTRA_COOLDOWN_PERIOD) {
            // motor_direction = 1;
            input_velocity = -fabs(reference_velocity);
            // turnStepsPerSecond(motor_speed, motor_direction);
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
    if (flex_cooldown_passed && omega_x > OMEGA_X_FLEX_THRESHOLD) {
        // disableMotor();
        input_velocity = 0;
        flex_motor_running = false;
        flex_cooldown_passed = false;
        flex_stop_timestamp = millis(); // Store stop time to enforce extra cooldown
    }


     // ANGLE-BASED CONDITIONS
     if (elbow_angle <= 5) {
        if(input_velocity < 0){//motor_direction == 1
            // disableMotor();
            input_velocity = 0;
        }
    }

    if (elbow_angle >= 88) {
        if(input_velocity > 0){ //motor_direction == 0
            // disableMotor();
            input_velocity = 0;
        }
    }

}