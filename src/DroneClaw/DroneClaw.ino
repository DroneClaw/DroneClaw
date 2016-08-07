/*
    DroneClaw copyright 2016
*/
#include <EventLoop.h>
#include <Servo.h>
#include <Wire.h>
#include "PID.hpp"
#include "MPU.hpp"

//#define DEBUG // Will enable debug code throught the program
#define DEBUG_MOTOR 1100

#define FL_ESC 0
#define FR_ESC 1
#define BL_ESC 2
#define BR_ESC 3
#define FL_ESC_PIN 5
#define FR_ESC_PIN 6
#define BL_ESC_PIN 10
#define BR_ESC_PIN 11
#define CLAW 3
#define SERIAL_BAUD 115200

EventLoop scheduler;
Servo servos[4] = {};
Servo claw;

struct {
  volatile unsigned long last_ping = 0;
  volatile int throttle = 0;
  volatile int pitch = 0;
  volatile int roll = 0;
  volatile int yaw = 0;
} drone;

/** The main control loop */
void control() {
  // caculate the pids
  PID pid;
  float pid_pitch = pid.pitch(drone.pitch);
  float pid_roll = pid.roll(drone.roll);
  float pid_yaw = pid.yaw(drone.yaw);
  if (drone.throttle < 1000) { // when throttle is less than 1000 disable escs
    servos[FR_ESC].write(0);
    servos[FL_ESC].write(0);
    servos[BR_ESC].write(0);
    servos[BL_ESC].write(0);
  } else { // write the data to the servos or graphs
    #ifdef DEBUG // print motor values to graphs
    int fl = drone.throttle - pid_pitch + pid_roll - pid_yaw;
    int fr = drone.throttle - pid_pitch - pid_roll + pid_yaw;
    int bl = drone.throttle + pid_pitch + pid_roll + pid_yaw;
    int br = drone.throttle + pid_pitch - pid_roll - pid_yaw;
    Serial.print(fr);
    Serial.print(",");
    Serial.print(br);
    Serial.print(",");
    Serial.print(fl);
    Serial.print(",");
    Serial.print(bl);
    Serial.println();
    // write the data to the servos
    servos[FR_ESC].writeMicroseconds(DEBUG_MOTOR);
    servos[BR_ESC].writeMicroseconds(DEBUG_MOTOR);
    servos[FL_ESC].writeMicroseconds(DEBUG_MOTOR);
    servos[BL_ESC].writeMicroseconds(DEBUG_MOTOR);
    #else
    // write the data to the servos
    servos[FR_ESC].writeMicroseconds(drone.throttle - pid_pitch - pid_roll + pid_yaw);
    servos[BR_ESC].writeMicroseconds(drone.throttle + pid_pitch - pid_roll - pid_yaw);
    servos[FL_ESC].writeMicroseconds(drone.throttle - pid_pitch + pid_roll - pid_yaw);
    servos[BL_ESC].writeMicroseconds(drone.throttle + pid_pitch + pid_roll + pid_yaw);
    #endif
  }
}

void fail_safe() {
  if (drone.last_ping-- < 1) {
    drone.throttle = 0;
    drone.pitch = 0;
    drone.roll = 0;
    drone.yaw = 0;
  }
}

void setup() {
  // Get the sensors and ect ready before bluetooth connection is established
  Serial.begin(SERIAL_BAUD);
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
  while (!Serial.available() || Serial.parseInt() != 0) {
    claw.write(38 * sin(x++  * 0.0125) + 83); // min of 45~, max of 120~, period of 258
    delay(10);
  }
  claw.write(45);
  // show the header of the program
  #ifdef DEBUG
  Serial.println("\n----- DroneClaw -----\n");
  #endif
  // Set up escs
  #ifdef DEBUG
  Serial.println("Setting up the escs");
  #endif
  servos[FR_ESC].attach(FR_ESC_PIN);
  servos[FL_ESC].attach(FL_ESC_PIN);
  servos[BR_ESC].attach(BR_ESC_PIN);
  servos[BL_ESC].attach(BL_ESC_PIN);
  // Handle the fail safe
  scheduler.repeat(fail_safe, 1, SECONDS);
}

void loop() {
  scheduler.process(); // The backbone of the system
}

