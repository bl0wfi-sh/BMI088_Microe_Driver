/*
    A library for PID based multi-copter rate control.
    This library runs 3 independant PID loops for control of a quadcopters main 3 axis of rotation pitch, roll, and yaw.
    
    Author      : Felix Blanco
    Create Time : July 2020
    Change Log  :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#ifndef __RATE_CONTROLLER_H__
#define __RATE_CONTROLLER_H__

#include <Arduino.h>

class RateControl {
  public:
    RateControl(float pitch_gains[], float roll_gains[], float yaw_gains[]);
    RateControl(void);
    void loopController(float sensor_states[], float desired_states[], float delta_t);
    void outputMixer(float max_motor_output, float min_motor_output);
    
  private:

    // Gains
    float r_p = 0; float r_i = 0; float r_d = 0;
    float p_p = 0; float p_i = 0; float p_d = 0;
    float y_p = 0; float y_i = 0; float y_d = 0;
    
    // Errors
    float roll_err = 0;
    float pitch_err = 0;
    float yaw_err = 0;

    float roll_prev_err = 0;
    float pitch_prev_err = 0;
    float yaw_prev_err = 0;
    
    float roll_err_int = 0;
    float pitch_err_int = 0;
    float yaw_err_int = 0;
    
    float roll_err_deriv = 0;
    float pitch_err_deriv = 0;
    float yaw_err_deriv = 0;

    // Controller Outputs
    float comm_roll = 0;
    float comm_pitch = 0;
    float comm_yaw = 0;

    // Motors
    float fl;     // Front Left
    float fr;     // Front Right
    float bl;     // Back Left
    float br;     // Back Right
};

extern RateControl mc_rc;

#endif
