#include "IMUCalibration.h"
#include <Arduino_LSM9DS1.h>

const int maxReadings = 1000; // Define the maximum number of readings for calibration
const int delayBetweenReadings = 10; // Delay between readings (milliseconds)

// Function to sort an array and return the median value
float calculateMedian(float readings[], int numReadings) {
  for (int i = 0; i < numReadings - 1; i++) {
    for (int j = i + 1; j < numReadings; j++) {
      if (readings[i] > readings[j]) {
        float temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }

  // Return the median value
  if (numReadings % 2 == 0) {
    // If even number of readings, take average of two middle values
    return (readings[numReadings / 2 - 1] + readings[numReadings / 2]) / 2;
  } else {
    // If odd number of readings, take the middle value
    return readings[numReadings / 2];
  }
}

void calibrateIMU(int numReadings, float &ax_offset, float &ay_offset, float &az_offset, float &gx_offset, float &gy_offset, float &gz_offset) {
  float ax_readings[maxReadings], ay_readings[maxReadings], az_readings[maxReadings];
  float gx_readings[maxReadings], gy_readings[maxReadings], gz_readings[maxReadings];

  int readingCount = 0;

  // Collect readings
  while (readingCount < numReadings) {
    float ax, ay, az;
    float gx, gy, gz;

    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      ax_readings[readingCount] = ax;
      ay_readings[readingCount] = ay;
      az_readings[readingCount] = az;
    }

    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      gx_readings[readingCount] = gx;
      gy_readings[readingCount] = gy;
      gz_readings[readingCount] = gz;
    }

    readingCount++;
    delay(delayBetweenReadings);
  }

  // Calculate median for each axis
  ax_offset = calculateMedian(ax_readings, numReadings);
  ay_offset = calculateMedian(ay_readings, numReadings);
  az_offset = calculateMedian(az_readings, numReadings);

  gx_offset = calculateMedian(gx_readings, numReadings);
  gy_offset = calculateMedian(gy_readings, numReadings);
  gz_offset = calculateMedian(gz_readings, numReadings);
}