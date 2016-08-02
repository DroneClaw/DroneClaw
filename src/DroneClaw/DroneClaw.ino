/*
    DroneClaw copyright 2016
*/
#include <EventLoop.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "MPU.hpp"

// Will enable debug code throught the program
//#define DEBUG

#define FL_ESC 0
#define FR_ESC 1
#define BL_ESC 2
#define BR_ESC 3
#define FL_ESC_PIN 5
#define FR_ESC_PIN 6
#define BL_ESC_PIN 10
#define BR_ESC_PIN 11
#define CLAW 3
#define BT_RX 8
#define BT_TX 9
#define PACKETS 4
#define BAUD 9600

EventLoop &scheduler = EventLoop::get();
Servo servos[4] = {};
SoftwareSerial bluetooth(BT_RX, BT_TX);
Servo claw;

struct {
  volatile int throttle = 0;
  volatile int pitch = 0;
  volatile int roll = 0;
} drone;

float p_error = 0;
float r_error = 0;
float i_pitch = 0;
float i_roll = 0;
float angle_pitch, angle_roll;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

/** The main control loop */
void control() {
  // when throttle is less than 1000 disable escs
  if (drone.throttle < 1000) {
    servos[FR_ESC].write(0);
    servos[FL_ESC].write(0);
    servos[BR_ESC].write(0);
    servos[BL_ESC].write(0);
    // reset pid
    p_error = 0;
    r_error = 0;
    i_pitch = 0;
    i_roll = 0;
    return;
  }
  // caculate the pids
  MPU mpu;
  mpu.gyro_x -= MPU::calibration()[0];
  mpu.gyro_y -= MPU::calibration()[1];
  mpu.gyro_z -= MPU::calibration()[2];
  // Gyro angle calculations
  angle_pitch += mpu.gyro_x * 0.0000611;
  angle_roll += mpu.gyro_y * 0.0000611;
  angle_pitch += angle_roll * sin(mpu.gyro_z * 0.000001066);
  angle_roll -= angle_pitch * sin(mpu.gyro_z * 0.000001066);
  // Accelerometer angle calculations
  long acc_total_vector = sqrt((mpu.accel_x * mpu.accel_x) + (mpu.accel_y * mpu.accel_y ) + (mpu.accel_z * mpu.accel_z));
  angle_pitch_acc = asin((float) mpu.accel_y /acc_total_vector) * 57.296;
  angle_roll_acc = asin((float) mpu.accel_x / acc_total_vector) * 57.296;
  if (set_gyro_angles) {
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  } else {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
  // derivative
  float d_pitch = angle_pitch_output - drone.pitch - p_error;
  float d_roll = angle_roll_output - drone.roll - r_error;
  // proportional
  p_error = angle_pitch_output - drone.pitch;
  r_error = angle_roll_output - drone.roll;
  // intergral
  i_pitch += p_error;
  i_roll += r_error;
  #define MAX_TILT 200
  #define P_GAIN 4.3
  #define I_GAIN 0.04
  #define D_GAIN 18.0
  float pid_pitch = (P_GAIN * p_error +  I_GAIN * i_pitch + D_GAIN * d_pitch);
  float pid_roll = (P_GAIN * r_error +  I_GAIN * i_roll + D_GAIN * d_roll);
  if (pid_pitch > MAX_TILT) {
    pid_pitch = MAX_TILT;
  } else if (pid_pitch < -MAX_TILT) {
    pid_pitch = -MAX_TILT;
  }
  if (pid_roll > MAX_TILT) {
    pid_roll = MAX_TILT;
  } else if (pid_roll < -MAX_TILT) {
    pid_roll = -MAX_TILT;
  }
  int fl = drone.throttle - pid_pitch + pid_roll,
  fr = drone.throttle - pid_pitch - pid_roll,
  bl = drone.throttle + pid_pitch + pid_roll,
  br = drone.throttle + pid_pitch - pid_roll;
  #ifdef DEBUG // print motor values to graphs
  Serial.print(fr);
  Serial.print(",");
  Serial.print(br);
  Serial.print(",");
  Serial.print(fl);
  Serial.print(",");
  Serial.print(bl);
  Serial.println();
  #define DEBUG_MOTOR 1100
  servos[FR_ESC].writeMicroseconds(DEBUG_MOTOR);
  servos[BR_ESC].writeMicroseconds(DEBUG_MOTOR);
  servos[FL_ESC].writeMicroseconds(DEBUG_MOTOR);
  servos[BL_ESC].writeMicroseconds(DEBUG_MOTOR);
  #else
  // write the data to the servos
  servos[FR_ESC].writeMicroseconds(fr);
  servos[BR_ESC].writeMicroseconds(br);
  servos[FL_ESC].writeMicroseconds(fl);
  servos[BL_ESC].writeMicroseconds(bl);
  #endif
}

#ifdef DEBUG
/** Print a message to the Serial only when debug is defined */
void println(String msg) {
  Serial.println(msg);
  bluetooth.println(msg);
}
#endif

void setup() {
  // Get the sensors and ect ready before bluetooth connection is established
  bluetooth.begin(BAUD);
  Serial.begin(BAUD);
  claw.attach(CLAW);
  MPU::init();
  // close claw slowly before calibrate
  for (unsigned int i = 0 ; i < 230; i++) {
    claw.write(38 * sin(i * 0.0125 + 5) + 83);
    delay(50);
  }
  // Calibrate the gyro
  MPU::calibrate();
  // Make sure the connection is established
  unsigned long x = 0;
  while ((!Serial.available() && !bluetooth.available()) || (Serial.parseInt() != 0 && bluetooth.parseInt() != 0)) {
    claw.write(38 * sin(x++  * 0.0125) + 83); // min of 45~, max of 120~, period of 258
    delay(10);
  }
  claw.write(45);
  // show the header of the program
  #ifdef DEBUG
  println("\n----- DroneClaw -----\n");
  #endif
  // Set up escs
  #ifdef DEBUG
  println("Setting up the escs");
  #endif
  servos[FR_ESC].attach(FR_ESC_PIN);
  servos[FL_ESC].attach(FL_ESC_PIN);
  servos[BR_ESC].attach(BR_ESC_PIN);
  servos[BL_ESC].attach(BL_ESC_PIN);
  // Handle the packets
  scheduler.repeat(process_packets, 250, MILLIS);
}

void loop() {
  scheduler.process(); // The backbone of the system
}
