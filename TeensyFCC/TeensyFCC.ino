
#include "BMI088.h"
#include "Servo.h"      // Included this so we can run Servo's @ 400Hz. Have not verified they actually run @ 400Hz.
#include "sbus.h"       // For reading in sbus information. 

// Motor Parameters and Variables
#define FLM   3
#define FRM   4
#define BRM   5
#define BLM   6
#define MIN_MOTOR_MS_VAL 1100       // At this value motors start spinning!
#define MAX_MOTOR_MS_VAL 1900
Servo FrontLeft, FrontRight, BackLeft, BackRight;

// IMU variables
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
int16_t temp = 0;
float veh_pitch, veh_roll, veh_yaw;
float gyro_roll, gyro_pitch, gyro_yaw;
float acc_pitch, acc_roll;

// PID Controller variables
float comm_roll = 0, comm_pitch = 0, comm_yaw = 0;

float roll_err = 0, pitch_err = 0, yaw_err = 0;
float roll_prev_err = 0, pitch_prev_err = 0, yaw_prev_err = 0;

float roll_err_int = 0, pitch_err_int = 0, yaw_err_int = 0;
float roll_err_deriv = 0, pitch_err_deriv = 0, yaw_err_deriv = 0;

float r_p = 2.4, r_i = 0, r_d = 0;      // PID - Roll
float p_p = 2.4, p_i = 0, p_d = 0;      // PID - Pitch
float y_p = 0, y_i = 0, y_d = 0;      // PI - Yaw

// RC receiver variables.
bfs::SbusRx sbus_rx(&Serial1);
std::array<int16_t, bfs::SbusRx::NUM_CH()> sbus_data;
#define MAX_CH_VAL  1693
#define MIN_CH_VAL  306
#define MID_CH_VAL  1387
#define ROLL_CH     1
#define PITCH_CH    2
#define THROTTLE_CH 3
#define YAW_CH      4
#define MODE_CH     5
#define ARM_CH      10

// Modes
// 0 - Off mode. LED blinks slow.
// 1 - RC Passthrough (no flight controller in the loop). LED blinks medium.
// 2 - Stabilize mode (with rate controller in the loop for pitch and roll, NO YAW). LED blinks fast!
int flight_mode;
bool state_updated;
bool armed;
const int ledpin = LED_BUILTIN;
int ledstate = LOW;
IntervalTimer modeTimer;

// Debug console!
IntervalTimer logTimer;

void logToConsole(){
  Serial.print(r_p);
  Serial.print(", ");
  Serial.print(veh_roll);
  Serial.print(", ");
  Serial.print(veh_pitch);
  Serial.print(", ");
  Serial.print(veh_yaw);
  Serial.print(", ");
  Serial.print(comm_roll);
  Serial.print(", ");
  Serial.print(comm_pitch);
  Serial.print(", ");
  Serial.println(comm_yaw);
}

void blinkLED(){
  if (ledstate == LOW) {
    ledstate = HIGH;
  } else {
    ledstate = LOW;
  }
  digitalWrite(ledpin, ledstate);
}

void setup(void) {

  // Set default operational mode to manual
  flight_mode = 2;
  armed = false;
  pinMode(ledpin, OUTPUT);
  modeTimer.begin(blinkLED, 500000);

  // Start logger process.
  logTimer.begin(logToConsole, 10000);      // Logging @ 100Hz
  
  // Setup sbus receiver.
  sbus_rx.Begin();

  // Setup motor PWM pins as outputs.
  FrontLeft.attach(FLM, 1000, 2000);
  FrontRight.attach(FRM, 1000, 2000);
  BackLeft.attach(BLM, 1000, 2000);
  BackRight.attach(BRM, 1000, 2000);

  // Sets ESC's to 0 throttle.
  FrontLeft.writeMicroseconds(1000);
  FrontRight.writeMicroseconds(1000);
  BackLeft.writeMicroseconds(1000);
  BackRight.writeMicroseconds(1000);
  
  Wire.begin();
  Serial.begin(1000000);
  
  // Waiting to see if IMU is connected.
  while (1) {
      if (bmi088.isConnection()) {
          bmi088.initialize();
          //Serial.println("BMI088 is connected");
          break;
      } else {
          Serial.println("BMI088 is not connected");
      }
  
      delay(2000);
  }  

  // Initialize gyro angles to accelerometer readings in the beginning.
  bmi088.getAcceleration(&ax, &ay, &az);  // g/s^2 
  float acc_tot_vect = sqrt((ax*ax) + (ay*ay) + (az*az));
  acc_roll = asin(ay/acc_tot_vect) * (180.0 /3.142);
  acc_pitch = asin(ax/acc_tot_vect) * (180.0 /3.142);
  gyro_roll = acc_roll;
  gyro_pitch = acc_pitch;
}

void loop(void) {

  unsigned long st_time = millis();

  // Read accelerometer
  bmi088.getAcceleration(&ax, &ay, &az);  // g/s^2
  bmi088.getGyroscope(&gx, &gy, &gz);     // deg/s
  temp = bmi088.getTemperature();

  // Estimating pitch, roll, and yaw angles by integrating and transfering pitch to roll and roll to pitch based on yaw.
  gyro_roll += (gx * .004);
  gyro_pitch += (-gy * .004);
  gyro_roll += gyro_pitch * sin(-gz * .004 * 3.142 / 180.0);
  gyro_pitch -= gyro_roll * sin(-gz * .004 * 3.142 / 180.0);
  gyro_yaw += (-gz * .004);
  
  // Estimating roll and pitch based on accelerometer.
  float acc_tot_vect = sqrt((ax*ax) + (ay*ay) + (az*az));
  acc_roll = asin(ay/acc_tot_vect) * (180.0 /3.142);
  acc_pitch = asin(ax/acc_tot_vect) * (180.0 /3.142);
  
  // Now lets use both the gyro and accel with a complimentary filter.
  veh_roll = (gyro_roll * 0.9995) + (acc_roll * 0.0005);
  veh_pitch = (gyro_pitch * 0.9995) + (acc_pitch * 0.0005);
  veh_yaw = gyro_yaw;

  // Read from SBUS to control quad states and read controller inputs.
  if (sbus_rx.Read()) {
    sbus_data = sbus_rx.ch();

    // Update armed state.
    if (map(sbus_data[ARM_CH-1], MIN_CH_VAL, MAX_CH_VAL, MIN_MOTOR_MS_VAL, MAX_MOTOR_MS_VAL) > 1500){
      armed = true;
    }else{
      armed = false;
      FrontLeft.writeMicroseconds(MIN_MOTOR_MS_VAL);
      FrontRight.writeMicroseconds(MIN_MOTOR_MS_VAL);
      BackLeft.writeMicroseconds(MIN_MOTOR_MS_VAL);
      BackRight.writeMicroseconds(MIN_MOTOR_MS_VAL);
    }

    // Update mode selection.
    if (map(sbus_data[MODE_CH-1], MIN_CH_VAL, MAX_CH_VAL, MIN_MOTOR_MS_VAL, MAX_MOTOR_MS_VAL) > 1500){
      flight_mode = 2;
      modeTimer.update(100000);
    }else if (map(sbus_data[MODE_CH-1], MIN_CH_VAL, MAX_CH_VAL, MIN_MOTOR_MS_VAL, MAX_MOTOR_MS_VAL) < 1500){
      flight_mode = 0;
      modeTimer.update(2000000);
    }else {
      flight_mode = 1;
      modeTimer.update(500000);
    }
  }

  // Switch between flight modes.
  if (flight_mode == 1 and armed == true)
  {
    int throttle = map(sbus_data[THROTTLE_CH-1], MIN_CH_VAL, MAX_CH_VAL, MIN_MOTOR_MS_VAL, MAX_MOTOR_MS_VAL);
    int pitch_pwm = map(sbus_data[PITCH_CH-1], MIN_CH_VAL, MAX_CH_VAL, -50, 50);      // All of these inputs are as deltas from throttle pwm value.
    int roll_pwm = map(sbus_data[ROLL_CH-1], MIN_CH_VAL, MAX_CH_VAL, -50, 50);        // ^
    int yaw_pwm = map(sbus_data[YAW_CH-1], MIN_CH_VAL, MAX_CH_VAL, -50, 50);          // ^
  
    // Mixer, allocation of input signal.
    FrontLeft.writeMicroseconds(throttle + roll_pwm + pitch_pwm + yaw_pwm);
    FrontRight.writeMicroseconds(throttle - roll_pwm + pitch_pwm - yaw_pwm);
    BackLeft.writeMicroseconds(throttle + roll_pwm - pitch_pwm - yaw_pwm);
    BackRight.writeMicroseconds(throttle - roll_pwm - pitch_pwm + yaw_pwm);
  }else if (flight_mode == 2 and armed == true)
  {
    // Stabilization mode. 
    // PID controller running on pitch and roll axis.
    int throttle = map(sbus_data[THROTTLE_CH-1], MIN_CH_VAL, MAX_CH_VAL, MIN_MOTOR_MS_VAL, MAX_MOTOR_MS_VAL);
    float des_pitch = map((float)sbus_data[PITCH_CH-1], (float)MIN_CH_VAL, (float)MAX_CH_VAL, -5.0, 5.0);        // Inputs are in desired degrees.
    float des_roll = map((float)sbus_data[ROLL_CH-1], (float)MIN_CH_VAL, (float)MAX_CH_VAL, -5.0, 5.0);          // ^
    float des_yaw = map((float)sbus_data[YAW_CH-1], (float)MIN_CH_VAL, (float)MAX_CH_VAL, -5.0, 5.0);            // ^

    // Signage defines direction we need to go in.
    roll_err = des_roll - veh_roll;
    pitch_err = des_pitch - veh_pitch;
    yaw_err = des_yaw - veh_yaw;

    // Put error through PID for each axis
    roll_err_int += (roll_prev_err + roll_err) * (.004 / 2.0);     // Trapazoidal integral estimation
    pitch_err_int += (pitch_prev_err + pitch_err) * (.004 / 2.0);  // ^
    yaw_err_int += (yaw_prev_err + yaw_err) * (.004 / 2.0);        // ^
    
    roll_err_deriv = (roll_err - roll_prev_err) / .004;            // Finite difference derivative estimation
    pitch_err_deriv = (pitch_err - pitch_prev_err) / .004;         // ^
    yaw_err_deriv = (yaw_err - yaw_prev_err) / .004;               // ^

    roll_prev_err = roll_err;                                      // Done with previous errors, updating them.
    pitch_prev_err = pitch_err;                                    // ^
    yaw_prev_err = yaw_err;                                        // ^

    // Computing commanded roll, pitch, and yaw.
    comm_roll = (r_p * roll_err) + (r_i * roll_err_int) + (r_d * roll_err_deriv);
    comm_pitch = (p_p * pitch_err) + (p_i * pitch_err_int) + (p_d * pitch_err_deriv);
    comm_yaw = (y_p * yaw_err) + (y_i * yaw_err_int) + (y_d * yaw_err_deriv);

    // Putting commanded roll, pitch, and yaw through mixer.
    // Error handling for commanding too little and too greate of PWM values.
    float fl = throttle + comm_roll + comm_pitch + comm_yaw;
    if (fl > MAX_MOTOR_MS_VAL) fl = MAX_MOTOR_MS_VAL;
    if (fl < MIN_MOTOR_MS_VAL) fl = MIN_MOTOR_MS_VAL;

    float fr = throttle - comm_roll + comm_pitch - comm_yaw;
    if (fr > MAX_MOTOR_MS_VAL) fr = MAX_MOTOR_MS_VAL;
    if (fr < MIN_MOTOR_MS_VAL) fr = MIN_MOTOR_MS_VAL;

    float bl = throttle + comm_roll - comm_pitch - comm_yaw;
    if (bl > MAX_MOTOR_MS_VAL) bl = MAX_MOTOR_MS_VAL;
    if (bl < MIN_MOTOR_MS_VAL) bl = MIN_MOTOR_MS_VAL;

    float br = throttle - comm_roll - comm_pitch + comm_yaw;
    if (br > MAX_MOTOR_MS_VAL) br = MAX_MOTOR_MS_VAL;
    if (br < MIN_MOTOR_MS_VAL) br = MIN_MOTOR_MS_VAL;

    // Sending safe commands to 
    FrontLeft.writeMicroseconds((int)fl);
    FrontRight.writeMicroseconds((int)fr);
    BackLeft.writeMicroseconds((int)bl);
    BackRight.writeMicroseconds((int)br);
  }

  unsigned long end_time = millis();
  unsigned long elapsed = end_time - st_time;

  if (elapsed > 4){
    Serial.println("ERROR LOOP TOOK TOO LONG!!!");
    modeTimer.update(10000);
  }else{
    delay(4 - elapsed); // System will run @ 250Hz
  }
}
