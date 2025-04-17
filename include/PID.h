#ifndef PID_H
#define PID_H

struct PID {
    double input_velocity = 0;
    double output_velocity = 0; // moet dit 21000 worden?
    double K_p = 10.0, K_i = 2.0, K_d = 10.0;  // Tuning parameters (pas aan voor optimale prestaties)
    double error_velocity = 0; // difference between setpoint and processVariable  
    double previous_error_velocity = 0; // error in previous iteration  
    double integral = 0; // integral of error  
    double derivative = 0; // derivative of error  
    double average_velocity = 0; // average velocity of the motor
};

void updatePID(PID& pid);

#endif
