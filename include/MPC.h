#ifndef MPH_H
#define MPH_H

struct MPC {
    const float elbow_radius = 42.426; // mm
    const float linear_velocity = 0.0003125; // mm/s --> lead/(microsteps per revolution) = 2/(2*16*200) = 0.0003125 mm/step
    float motor_speed_MPC = 0.0; // motor speed in degrees/s
    const float d = 30.0; // afstand van M naar A
    const float h = 60.0; // afstand van A naar B
}

void updateMPC(MPC& mpc);

#endif