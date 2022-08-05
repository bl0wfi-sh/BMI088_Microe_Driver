/*
    A library for PID based multi-copter control.
    This library runs 3 independant PID loops for control of a quadcopters main 3 axis of rotation pitch, roll, and yaw.
    This class can be used multiple times to create cascaded controllers.

    Example:
    PIDControl(Attitude) -> PIDControl(Rate)
    
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

#include "PIDController.h"

// Format of gains should be [P, I, D]
PIDControl::PIDControl(std::array<float, 3> roll_gains, std::array<float, 3> pitch_gains, std::array<float, 3> yaw_gains)
{
  this->r_p = roll_gains[0];
  this->r_i = roll_gains[1];
  this->r_d = roll_gains[2];
  
  this->p_p = pitch_gains[0];
  this->p_i = pitch_gains[1];
  this->p_d = pitch_gains[2];

  this->y_p = yaw_gains[0];
  this->y_i = yaw_gains[1];
  this->y_d = yaw_gains[2];
}

PIDControl::PIDControl(void)
{
  // Nothing is done here.
}

// States should be in the following format [d_x, d_y, d_z], [roll, pitch, yaw].
// Units of states are deg/s for all d_x, d_y, and d_z.
// Units of delta_t is seconds.
std::array<float, 3> PIDControl::loopController(std::array<float, 3> sensor_states, std::array<float, 3> desired_states, float delta_t)
{
  std::array<float, 3> comm_out;
  
  this->roll_err = desired_states[0] - sensor_states[0];                              // Roll
  this->pitch_err = desired_states[1] - sensor_states[1];                             // Pitch
  this->yaw_err = desired_states[2] - sensor_states[2];                               // Yaw

  // Put error through PID for each axis
  this->roll_err_int += (this->roll_prev_err + this->roll_err) * (delta_t / 2.0);     // Trapazoidal integral estimation
  this->pitch_err_int += (this->pitch_prev_err + this->pitch_err) * (delta_t / 2.0);  // ^
  this->yaw_err_int += (this->yaw_prev_err + this->yaw_err) * (delta_t / 2.0);        // ^
  
  this->roll_err_deriv = (this->roll_err - this->roll_prev_err) / delta_t;            // Finite difference derivative estimation
  this->pitch_err_deriv = (this->pitch_err - this->pitch_prev_err) / delta_t;         // ^
  this->yaw_err_deriv = (this->yaw_err - this->yaw_prev_err) / delta_t;               // ^

  this->roll_prev_err = this->roll_err;                                               // Done with previous errors, updating them.
  this->pitch_prev_err = this->pitch_err;                                             // ^
  this->yaw_prev_err = this->yaw_err;                                                 // ^

  // Computing commanded roll, pitch, and yaw.
  comm_out[0] = (this->r_p * this->roll_err) + (this->r_i * this->roll_err_int) + (this->r_d * this->roll_err_deriv);
  comm_out[1] = (this->p_p * this->pitch_err) + (this->p_i * this->pitch_err_int) + (this->p_d * this->pitch_err_deriv);
  comm_out[2] = (this->y_p * this->yaw_err) + (this->y_i * this->yaw_err_int) + (this->y_d * this->yaw_err_deriv);

  return comm_out;
}

std::array<float, 9> PIDControl::getGains(){  // [roll, pitch, yaw]
  std::array<float, 9> gains = {r_p, r_i, r_d,
                                p_p, p_i, p_d,
                                y_p, y_i, y_d};
  return gains;
}
void PIDControl::setGains(std::array<float, 3> roll_gains, std::array<float, 3> pitch_gains, std::array<float, 3> yaw_gains){
  this->r_p = roll_gains[0];
  this->r_i = roll_gains[1];
  this->r_d = roll_gains[2];
  
  this->p_p = pitch_gains[0];
  this->p_i = pitch_gains[1];
  this->p_d = pitch_gains[2];

  this->y_p = yaw_gains[0];
  this->y_i = yaw_gains[1];
  this->y_d = yaw_gains[2];
}
