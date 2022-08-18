/*
    A library for PID based multi-copter control.
    This library runs 3 independant PID loops for control of a quadcopters main 3 axis of rotation pitch, roll, and yaw.
    This class can be used multiple times to create cascaded controllers.

    Example:
    PIDControl(Attitude) -> PIDControl(Rate)
    
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

#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include <array>

class PIDControl {
  public:
    PIDControl(std::array<float, 3> roll_gains, std::array<float, 3> pitch_gains, std::array<float, 3> yaw_gains);
    PIDControl(void);
    std::array<float, 3> loopController(std::array<float, 3> sensor_states, std::array<float, 3> desired_states, float delta_t);

    // Utility functions for gains.
    std::array<float, 9> getGains();  // [roll, pitch, yaw]
    void setGains(std::array<float, 3> roll_gains, std::array<float, 3> pitch_gains, std::array<float, 3> yaw_gains);
    
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
};

#endif
