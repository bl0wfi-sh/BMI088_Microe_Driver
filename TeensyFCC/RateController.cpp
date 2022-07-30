#include "RateController.h"

// Format of gains should be [P, I, D]
RateControl::RateControl(float pitch_gains[], float roll_gains[], float yaw_gains[])
{
  this->p_p = pitch_gains[0];
  this->p_i = pitch_gains[1];
  this->p_d = pitch_gains[2];

  this->r_p = roll_gains[0];
  this->r_i = roll_gains[1];
  this->r_d = roll_gains[2];

  this->y_p = yaw_gains[0];
  this->y_i = yaw_gains[1];
  this->y_d = yaw_gains[2];
}

RateControl::RateControl(void)
{
  // Nothing is done here.
}


// States should be in the following format [d_x, d_y, d_z].
// Units of states are rad/s for all d_x, d_y, and d_z.
// Units of delta_t is seconds.
void RateControl::loopController(float sensor_states[], float desired_states[], float delta_t)
{
  // Nothing is done here.
}

// Both max_motor_output and min_motor_output are unit less.
// They are simply used to saturate motor output to a desired min/max value.
// Typically these variables are in PWM micro-seconds for each motor ESC.
void RateControl::outputMixer(float max_motor_output, float min_motor_output)
{
  // Nothing is done here.
}

RateControl mc_rc;
