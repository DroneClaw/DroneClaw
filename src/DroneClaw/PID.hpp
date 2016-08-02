/*
    DroneClaw copyright 2016
*/

#ifndef _PID_
#define _PID_

#include "MPU.hpp"

#define FREQUENCY 0.0000611
#define FREQUENCY_RAD FREQUENCY * DEG_TO_RAD

#define MAX_TILT 400
#define P_GAIN 4.3
#define I_GAIN 0.04
#define D_GAIN 18.0

class PID {
  private:
    // mpu
    static boolean _gyro_angles;
    static float _pitch_angle;
    static float _pitch_accelerometer;
    static float _pitch;
    static float _roll_angle;
    static float _roll_accelerometer;
    static float _roll;
    // pid
    static float _pid_pitch[2];
    static float _pid_roll[2];
    /** The PID math */
    inline int pid(float &proportional, float &intergral, const float &delta, const float &mpu) {
      float derivative = mpu - delta - proportional;
      proportional = mpu - delta;
      intergral += proportional;
      float pid = (P_GAIN * proportional +  I_GAIN * intergral + D_GAIN * derivative);
      if (pid > MAX_TILT) {
        return MAX_TILT;
      } else if (pid < -MAX_TILT) {
        return -MAX_TILT;
      }
      return pid;
    }
  public:
    inline PID() {
      MPU mpu;
      Vector<int> gyro = mpu.gyro();
      Vector<long> accel = mpu.accelerometer();
      // Gyro angle calculations
      _pitch_angle += gyro.x * FREQUENCY;
      _roll_angle += gyro.y * FREQUENCY;
      _pitch_angle += _roll_angle * sin(gyro.z * FREQUENCY_RAD);
      _roll_angle -= _pitch_angle * sin(gyro.z * FREQUENCY_RAD);
      // Accelerometer angle calculations
      long accelerometer_total = sqrt(sq(accel.x) + sq(accel.y) + sq(accel.z));
      _pitch_accelerometer = asin((float) accel.y / accelerometer_total) * RAD_TO_DEG;
      _roll_accelerometer = asin((float) accel.x / accelerometer_total) * RAD_TO_DEG;
      if (_gyro_angles) {
        _pitch_angle = _pitch_angle * 0.9996 + _pitch_accelerometer * 0.0004;
        _roll_angle = _roll_angle * 0.9996 + _roll_accelerometer * 0.0004;
      } else {
        _pitch_angle = _pitch_accelerometer;
        _roll_angle = _roll_accelerometer;
        _gyro_angles = true;
      }
      // To dampen the pitch and roll angles a complementary filter is used
      _pitch = _pitch * 0.9 + _pitch_angle * 0.1;
      _roll = _roll * 0.9 + _roll_angle * 0.1;
    }
    /** Caculate the PID for the pitch */
    inline float pitch(const float &pitch) {
      return pid(_pid_pitch[0], _pid_pitch[1], pitch, _pitch);
    }
    /** Caculate the PID for the roll */
    inline float roll(const float &roll) {
      return pid(_pid_roll[0], _pid_roll[1], roll, _roll);
    }
    /** Reset PID where needed */
    inline static void reset_pid() {
      _pid_pitch[2] = {};
      _pid_roll[2] = {};
    }
};

boolean PID::_gyro_angles = false;
float PID::_pitch_angle = 0;
float PID::_pitch_accelerometer = 0;
float PID::_pitch = 0;
float PID::_roll_angle = 0;
float PID::_roll_accelerometer = 0;
float PID::_roll = 0;
float PID::_pid_pitch[2] = {};
float PID::_pid_roll[2] = {};

#endif

