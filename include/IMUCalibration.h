#ifndef IMUCALIBRATION_H
#define IMUCALIBRATION_H

void calibrateIMU(int numReadings, float &ax_offset, float &ay_offset, float &az_offset, float &gx_offset, float &gy_offset, float &gz_offset);
float correctGyroDrift(float current_gyro_value);

// Constants
extern const float driftCompensationRate;

#endif