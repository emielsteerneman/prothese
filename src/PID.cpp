#include "PID.h"

void updatePID(PID& pid){
    pid.error_velocity = fabs(pid.input_velocity) - fabs(pid.average_velocity);  
    pid.integral += pid.error_velocity;  
    pid.derivative = pid.error_velocity - pid.previous_error_velocity;  
    pid.output_velocity = pid.K_p * pid.error_velocity + pid.K_i * pid.integral + pid.K_d * pid.derivative;  
    pid.previous_error_velocity = pid.error_velocity; 
}
