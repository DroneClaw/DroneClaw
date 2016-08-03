/*
    DroneClaw copyright 2016
*/

#include "Packet.hpp"

#define PACKETS 6

/** The packets the drone knows how to handle */
Packet packets[] = {
  // Ping packet used to make sure there is a connection
  Packet(0x00, [] (Stream &data) {
    // todo if last heart beat fails do something
  }),
  // Prime and arm packet, echo packet
  Packet(0x01, [] (Stream &data) {
    static boolean init = false;
    if (!init) {
      servos[FR_ESC].write(1);
      servos[FL_ESC].write(1);
      servos[BR_ESC].write(1);
      servos[BL_ESC].write(1);
      scheduler.repeat(control, 4, MILLIS);
    }
  }),
  // Send the pos to the claw
  Packet(0x02, [] (Stream &data) {
    #ifdef DEBUG
    int pos = data.parseInt();
    println("Claw Position: " + String(pos));
    claw.write(pos);
    #else
    claw.write(data.parseInt());
    #endif
  }),
  // Send data to all escs
  Packet(0x03, [] (Stream &data) {
    drone.throttle = data.parseInt();
    drone.roll = data.parseInt();
    drone.pitch = data.parseInt();
    drone.yaw = data.parseInt();
  }),
  // Show pitch roll and yaw to the current stream
  Packet(0x04, [] (Stream &data) {
    PID pid;
    float* raw_data = pid.to_vector();
    data.print(raw_data[X]); // pitch
    data.print(",");
    data.print(raw_data[Y]); // roll
    data.print(",");
    data.println(raw_data[Z]); // yaw
  }),
  // Show pitch roll and yaw to the current stream
  Packet(0x05, [] (Stream &data) {
    PID pid;
    data.print(pid.pitch(drone.pitch)); // pitch
    data.print(",");
    data.print(pid.roll(drone.roll)); // roll
    data.print(",");
    data.println(pid.yaw(drone.yaw)); // yaw
  }),
};

/** Will process the incomming packets */
void process_packets() {
  byte packet = -1;
  if (bluetooth.available()) {
    packet = bluetooth.parseInt(); // Get the packet id
  } else if (Serial.available()) {
    packet = Serial.parseInt();
  }
  if (packet >= 0 && packet < PACKETS) {
    #ifdef DEBUG
    println("Packet ID: " + String(packet));
    #endif
    if (bluetooth.available()) {
      packets[packet].decode(bluetooth); // Lets the packet process the rest of the data
    } else {
      packets[packet].decode(Serial); // Lets the packet process the rest of the data
    }
  }
  // toggle the boolean to show invalid packets
  #if defined(DEBUG) && false
  else {
    println("Not a valid packet id");
  }
  #endif
}

