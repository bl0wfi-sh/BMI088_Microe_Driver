/*
    A library for Model-Free based multi-copter control.
    This library runs 3 independant Model Free loops for control of a quadcopters main 3 axis of rotation pitch, roll, and yaw.
    This class can be used multiple times to create cascaded controllers.

    Based on the work done by 'Maison Roland Cloutare' -> https://arxiv.org/pdf/2008.00681.pdf

    Example:
    MFController(something) -> MFController(Something Else)
    
    Author      : Felix Blanco
    Create Time : July 2022
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

#ifndef __MF_CONTROLLER_H__
#define __MF_CONTROLLER_H__

#include <array>

class MFControl {
  public:
    MFControl(std::array<float, 4> roll_params, std::array<float, 4> pitch_params, std::array<float, 4> yaw_params);
    MFControl(void);
    std::array<float, 3> loopController(std::array<float, 6> sensor_states, std::array<float, 3> desired_states, float delta_t);

    // Utility functions for gains.
    std::array<float, 3> getAlphas();  // [roll, pitch, yaw] loop alphas.
    void setAlphas(std::array<float, 3> new_alphas);

    std::array<float, 6> getGains();  // [roll, pitch, yaw] loop error gains.
    void setGains(std::array<float, 6> new_gains);

    std::array<float, 3> getFhatPeriod();  // [roll, pitch, yaw] loop Fhat estimation periods.
    void setFhatPeriod(std::array<float, 3> new_period);
    
  private:

    // Gains, Alphas, Fhat periods.
    float r_p = 0, r_d = 0, r_a = 1, r_t = .02;
    float p_p = 0, p_d = 0, p_a = 1, p_t = .02;
    float y_p = 0, y_d = 0, y_a = 1, y_t = .02;

    // Setpoint Variables
    float prev_roll_setpoint = 0, roll_setpoint_dot = 0, prev_roll_setpoint_dot = 0, roll_setpoint_dot_dot = 0;
    float prev_pitch_setpoint = 0, pitch_setpoint_dot = 0, prev_pitch_setpoint_dot = 0, pitch_setpoint_dot_dot = 0;
    float prev_yaw_setpoint = 0, yaw_setpoint_dot = 0, prev_yaw_setpoint_dot = 0, yaw_setpoint_dot_dot = 0;

    // Fhat Variables
    float roll_dot_dot = 0, prev_roll_dot = 0, new_fhat_roll = 0, estimating_fhat_roll = 0, prev_roll_output = 0;
    float pitch_dot_dot = 0, prev_pitch_dot = 0, new_fhat_pitch = 0, estimating_fhat_pitch = 0, prev_pitch_output = 0;
    float yaw_dot_dot = 0, prev_yaw_dot = 0, new_fhat_yaw = 0, estimating_fhat_yaw = 0, prev_yaw_output = 0;
    float fhat_estimation_elapsed_time = 0;

    // Rolling average buffer for setpoints.
    std::array<float, 10> input_roll;
    std::array<float, 10> input_pitch;
    std::array<float, 10> input_yaw;
};

#endif
