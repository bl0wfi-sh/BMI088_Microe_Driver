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
#include "MFController.h"

// Parameters should come in the following format.
// roll_params = [p, d, alpha, fhat_period]
MFControl::MFControl(std::array<float, 4> roll_params, std::array<float, 4> pitch_params, std::array<float, 4> yaw_params)
{
    this->r_p = roll_params[0];
    this->r_d = roll_params[1];
    this->r_a = roll_params[2];
    this->r_t = roll_params[3];

    this->p_p = pitch_params[0];
    this->p_d = pitch_params[1];
    this->p_a = pitch_params[2];
    this->p_t = pitch_params[3];

    this->y_p = yaw_params[0];
    this->y_d = yaw_params[1];
    this->y_a = yaw_params[2];
    this->y_t = yaw_params[3];
}

// The meat of the controller.
// Sensor_status = [roll_rate, pitch_rate, yaw_rate, roll_angle, pitch_angle, yaw_angle]
// Desired States = [roll, pitch, yaw] angles.
// Delat T = control loop frequency in seconds.
std::array<float, 3> MFControl::loopController(std::array<float, 6> sensor_states, std::array<float, 3> desired_states, float delta_t)
{
  std::array<float, 3> outputs;

  //********************* Calculating e and e_dot for all 3 axis. *********************
  // ROLL
  float roll_error = desired_states[0] - sensor_states[3];                          // Calculating e
  roll_setpoint_dot = (desired_states[0] - prev_roll_setpoint) / delta_t;           // Calculating yr_dot
  prev_roll_setpoint = desired_states[0];                                           // Updating previous setpoint value to current setpoint value.
  float roll_error_dot = roll_setpoint_dot - sensor_states[0];                      // Calculating e_dot
  // PITCH
  float pitch_error = desired_states[1] - sensor_states[4];                          // Calculating e
  pitch_setpoint_dot = (desired_states[1] - prev_pitch_setpoint) / delta_t;          // Calculating yr_dot
  prev_pitch_setpoint = desired_states[1];                                           // Updating previous setpoint value to current setpoint value.
  float pitch_error_dot = pitch_setpoint_dot - sensor_states[1];                     // Calculating e_dot
  // YAW
  float yaw_error = desired_states[2] - sensor_states[5];                          // Calculating e
  yaw_setpoint_dot = (desired_states[2] - prev_yaw_setpoint) / delta_t;          // Calculating yr_dot
  prev_yaw_setpoint = desired_states[2];                                           // Updating previous setpoint value to current setpoint value.
  float yaw_error_dot = yaw_setpoint_dot - sensor_states[2];                     // Calculating e_dot
  
  //********************* Calculating yr_dot_dot for all 3 axis *********************
  roll_setpoint_dot_dot = (roll_setpoint_dot - prev_roll_setpoint_dot) / delta_t;
  prev_roll_setpoint_dot = roll_setpoint_dot;
  
  pitch_setpoint_dot_dot = (pitch_setpoint_dot - prev_pitch_setpoint_dot) / delta_t;
  prev_pitch_setpoint_dot = pitch_setpoint_dot;
  
  yaw_setpoint_dot_dot = (yaw_setpoint_dot - prev_yaw_setpoint_dot) / delta_t;
  prev_yaw_setpoint_dot = yaw_setpoint_dot;

  //********************* Calculating u(t) based on above calculations. *********************
  prev_roll_output = (new_fhat_roll - roll_setpoint_dot_dot + (r_p * roll_error) + (r_d * roll_error_dot)) / (-r_a);
  outputs[0] = prev_roll_output;

  prev_pitch_output = (new_fhat_pitch - pitch_setpoint_dot_dot + (p_p * pitch_error) + (p_d * pitch_error_dot)) / (-p_a);
  outputs[1] = prev_pitch_output;
  
  prev_yaw_output = (new_fhat_yaw - yaw_setpoint_dot_dot + (y_p * yaw_error) + (y_d * yaw_error_dot)) / (-y_a);
  outputs[2] = prev_yaw_output;

  //********************* Estimating new Fhats. ********************* 
  roll_dot_dot = (sensor_states[0] - prev_roll_dot) / delta_t;
  prev_roll_dot = sensor_states[0];
  estimating_fhat_roll += (roll_dot_dot - (r_a * prev_roll_output)) * delta_t;

  pitch_dot_dot = (sensor_states[1] - prev_pitch_dot) / delta_t;
  prev_pitch_dot = sensor_states[1];
  estimating_fhat_pitch += (pitch_dot_dot - (p_a * prev_pitch_output)) * delta_t;

  yaw_dot_dot = (sensor_states[2] - prev_yaw_dot) / delta_t;
  prev_yaw_dot = sensor_states[2];
  estimating_fhat_yaw += (yaw_dot_dot - (y_a * prev_yaw_output)) * delta_t;

  fhat_estimation_elapsed_time += delta_t;

  if (fhat_estimation_elapsed_time > r_t)
  {
    // Time to pop out a new Fhat estimation for all 3 axis.
    new_fhat_roll = estimating_fhat_roll / r_t;
    new_fhat_pitch = estimating_fhat_pitch / p_t;
    new_fhat_yaw = estimating_fhat_yaw / y_t;

    estimating_fhat_roll = 0;
    estimating_fhat_pitch = 0;
    estimating_fhat_yaw = 0;

    fhat_estimation_elapsed_time = 0;
  }

  return outputs;
}

// Utility functions for gains.
std::array<float, 3> MFControl::getAlphas();  // [roll, pitch, yaw] loop alphas.
void MFControl::setAlphas(std::array<float, 3> new_alphas);

std::array<float, 6> MFControl::getGains();  // [roll, pitch, yaw] loop error gains.
void MFControl::setGains(std::array<float, 6> new_gains);

std::array<float, 3> MFControl::getFhatPeriod();  // [roll, pitch, yaw] loop Fhat estimation periods.
void MFControl::setFhatPeriod(std::array<float, 3> new_period);
