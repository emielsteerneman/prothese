import matplotlib.pyplot as plt
import numpy as np

reference_velocity = 12.5 # angular velocity in degrees/s
elbow_radius = 42.426 # radius in mm
microsteps = 32 # 16 times 2 for PWM
lead_ballsrew = 2 # mm
steps_per_revolution = 200 # steps per revolution of the motor, 1.8 degrees
elbow_angle = np.arange(5,90,1) # elbow angle in degrees
theta = np.arange(5,90,1) # elbow angle in degrees


linear_velocity  = 2/(200*32) # mm/s per microstep
# angular_velocity = (motor_speed * linear_velocity)/(elbow_radius*np.cos(np.radians(45-elbow_angle)))    # radians/s

# rewriting the equation to find motor speed
# motor_speed * linear_velocity = angular_velocity * elbow_radius * np.cos(np.radians(45-elbow_angle)
# motor_speed = (angular_velocity * elbow_radius * np.cos(np.radians(45-elbow_angle)) / linear_velocity
motor_speed_1 = (np.radians(reference_velocity) * elbow_radius * np.cos(np.radians(45-elbow_angle))) / linear_velocity ## deze klopt nu ook!


# numpy array of angles in degrees from 5 to 90, in steps of 1 degree
# theta = np.arange(5, 91, 1) # in degrees

motor_speed_2 = (reference_velocity * (42.426 * np.pi* np.cos(np.radians(45-theta)))) / 0.05625 ### deze klopt!



# plot motorspeeds vs angles
plt.plot(elbow_angle, motor_speed_1)
plt.xlabel("Angle (degrees)")
plt.ylabel("Motor speed (steps/s)")
plt.title("Motor speed vs angle at 12.5 degrees/s")
plt.grid()
plt.show()
