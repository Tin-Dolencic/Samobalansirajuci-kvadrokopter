#include <Wire.h>
#include <Servo.h>

Servo front_left_prop;
Servo front_right_prop;
Servo back_left_prop;
Servo back_right_prop;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, time, timePrev;
const float rad_to_deg = 180.0 / 3.141592654;

float error_pitch, error_roll, previous_error_pitch, previous_error_roll;
float pid_p_pitch, pid_i_pitch, pid_d_pitch;
float pid_p_roll, pid_i_roll, pid_d_roll;
float PID_pitch, PID_roll;

double kp_pitch = 6, ki_pitch = 0.01, kd_pitch = 0.4;
double kp_roll = 6, ki_roll = 0.01, kd_roll = 0.4;

double throttle = 1250;
const float desired_pitch = 0;
const float desired_roll = 0;

void setup() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);

  front_left_prop.attach(3);
  front_right_prop.attach(5);
  back_left_prop.attach(6);
  back_right_prop.attach(9);

  time = millis();

  front_left_prop.writeMicroseconds(1000);
  front_right_prop.writeMicroseconds(1000);
  back_left_prop.writeMicroseconds(1000);
  back_right_prop.writeMicroseconds(1000);
  delay(7000);
}

void loop() {
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000.0;

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6);
  Acc_rawX = Wire.read() << 8 | Wire.read();
  Acc_rawY = Wire.read() << 8 | Wire.read();
  Acc_rawZ = Wire.read() << 8 | Wire.read();

  Acceleration_angle[0] = atan(Acc_rawY / 16384.0 / sqrt(pow(Acc_rawX / 16384.0, 2) + pow(Acc_rawZ / 16384.0, 2))) * rad_to_deg;
  Acceleration_angle[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow(Acc_rawY / 16384.0, 2) + pow(Acc_rawZ / 16384.0, 2))) * rad_to_deg;

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4);
  Gyr_rawX = Wire.read() << 8 | Wire.read();
  Gyr_rawY = Wire.read() << 8 | Wire.read();

  Gyro_angle[0] = Gyr_rawX / 131.0;
  Gyro_angle[1] = Gyr_rawY / 131.0;

  Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.02 * Acceleration_angle[0];
  Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1] * elapsedTime) + 0.02 * Acceleration_angle[1];

  error_pitch = Total_angle[1] - desired_pitch;
  error_roll = Total_angle[0] - desired_roll;

  pid_p_pitch = kp_pitch * error_pitch;
  if (abs(error_pitch) < 3) pid_i_pitch += ki_pitch * error_pitch;
  pid_d_pitch = kd_pitch * (error_pitch - previous_error_pitch) / elapsedTime;
  PID_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;

  pid_p_roll = kp_roll * error_roll;
  if (abs(error_roll) < 3) pid_i_roll += ki_roll * error_roll;
  pid_d_roll = kd_roll * (error_roll - previous_error_roll) / elapsedTime;
  PID_roll = pid_p_roll + pid_i_roll + pid_d_roll;

  PID_pitch = constrain(PID_pitch, -1000, 1000);
  PID_roll = constrain(PID_roll, -1000, 1000);

  int pwm_fl = throttle + PID_pitch + PID_roll;
  int pwm_fr = throttle + PID_pitch - PID_roll;
  int pwm_bl = throttle - PID_pitch + PID_roll;
  int pwm_br = throttle - PID_pitch - PID_roll;

  pwm_fl = constrain(pwm_fl, 1000, 1800);
  pwm_fr = constrain(pwm_fr, 1000, 1800);
  pwm_bl = constrain(pwm_bl, 1000, 1800);
  pwm_br = constrain(pwm_br, 1000, 1800);

  front_left_prop.writeMicroseconds(pwm_fl);
  front_right_prop.writeMicroseconds(pwm_fr);
  back_left_prop.writeMicroseconds(pwm_bl);
  back_right_prop.writeMicroseconds(pwm_br);

  previous_error_pitch = error_pitch;
  previous_error_roll = error_roll;
}