# Teensy FCC
Teensy FCC is a Teensy 3.2 based experimental flight controller. Consisting of 4 main electronic components; Teensy 3.2 (Compute), MIKROE 13DOF Click (Sensors), SBUS RC receiver (Control), and a breadboard (Power Distribution).

Firmware has been developed through the Arduino development environment to enable easy plug and play of algorithms, sensor drivers, and other supporting hardware libraries such as the SBUS/Servo library.

## MIKROE 13DOF Click Board
13DOF click is based on the BME680, BMM150, and the BMI088 - a small, versatile 6DoF sensor module from Bosch purpose built for drone applications.
### Additional Sensor Info
- BMI088: I2C (400Khz), Accel Add (0x18), Gyro Add (0x68)

## Software Overview
The flight control software is made up of 5 sections; RC Receiver functions, Flight Modes, State Estimation, Vehicle Control Algorithms, and Loggers. The most interesting of which are State Estimation, and Vehicle Control Algorithms.

### State Estimation
Currently the state estimation algorithms are used to estimate the vehicles attitude. No position estimators are available at this time. Both estimation algorithms are running although the complimentary filter outputs are the ones actually being used by the controller at the moment.

- Kalman Filter: Kalman.cpp, Kalman.hpp. Currently outputs an estimation of pitch and roll based on accelerometer (attitude) and gyro (angular rates) readings.
- Complimentary Filter: Complimentary.cpp, Complimentary.hpp. Same as Kalman Filter. Complimentary filter is a lot easier to understand and produces results just as accurate as the Kalman filter.

### Vehicle Control Algorithms
Two control architectures are implemented at this time; Cascaded PID loop controller, Model-free attitude controller.
- Cascaded PID's: This is an implementation of the standard practice for multi-copter control. Similar more sophisticated implementations are found in PX4 and Ardupilot.
    - Gyro Rate Controller: The inner most PID controller whos sole purpose is to maintain desired angular rates. 3 PID loops are used in this implementation, one for each axis of rotation (pitch, roll, yaw). When flying your quadcopter in 'Acro' mode, only this controller is typically running. Difficult to control in this mode.
    - Attitude Controller: This controller wraps the Gyro Rate Controller. What this means is that the output of this controller is the input to the Gyro Rate controller. The purpose of this controller is to maintain desired attitude. In this implementation all 3 attitude controllers (pitch, roll, yaw) are simple P controllers.
    - Tunning recommendations. Tune Gyro Rate controllers first in pitch, roll, yaw with attitude controller OFF! Once desired dynamics are achieved enable the attitude controller loops and tune in pitch, roll, yaw.
- Model-Free Controller: Experimental! Read source code. Not fully functional or understood just yet. DO NOT FLY THIS MODE!

### Loop Frequencies
- State Estimation:
    - #define ESTIMATION_TIME (main) - Change this to change estimation loop time.
    - Default = 500Hz
- Vehicle Control Algorithm:
    - #define LOOP_TIME (main) - Change this to change controller loop time.
    - Default = 400Hz
- PWM Refresh Rate:
    - #define REFRESH_INTERVAL (Servo.h)
    - Default - 454Hz FREQUENCY MUST BE LARGER THAN CONTROL ALGORITHM!
- Logger:
    - #define LOGGER_TIME (main)
    - Default - 100Hz