#include "MPC.h"

void updateMPC(MPC& mpc){
    float theta = convertToRadians(elbow_angle+45);
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);

    float numerator = fabs(
        (30.0 - mpc.elbow_radius * sin_theta) * (mpc.elbow_radius * cos_theta) +
        mpc.elbow_radius * sin_theta * (60.0 + mpc.elbow_radius * cos_theta)
    );

    float denominator = sqrt(
        pow((30.0 - mpc.elbow_radius * sin_theta), 2) +
        pow((60.0 + mpc.elbow_radius * cos_theta), 2)
    );

    float distance = numerator / denominator;

    mpc.motor_speed_MPC = ((convertToRadians(fabs(reference_velocity)) * distance) / mpc.linear_velocity);
}