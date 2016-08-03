/*
    DroneClaw copyright 2016
*/
#include <EventLoop.h>
#include <Servo.h>
#include <SoftwareSerial.h>
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
#define BT_RX 8
#define BT_TX 9
#define BLUETOOTH_BAUD 9600
#define SERIAL_BAUD 57600

EventLoop scheduler;
Servo servos[4] = {};
SoftwareSerial bluetooth(BT_RX, BT_TX);
Servo claw;

struct {
  volatile int throttle;
  volatile int pitch;
  volatile int roll;
  volatile int yaw;
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

#ifdef DEBUG
/** Print a message to the Serial only when debug is defined */
void println(String msg) {
  Serial.println(msg);
  bluetooth.println(msg);
}
#endif

void setup() {
  // Get the sensors and ect ready before bluetooth connection is established
  bluetooth.begin(BLUETOOTH_BAUD);
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
  while ((!Serial.available() && !bluetooth.available()) || (Serial.parseInt() != 0 || bluetooth.parseInt() != 0)) {
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

