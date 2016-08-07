/*
    DroneClaw copyright 2016
*/

/** Create a packet type that is just a function that accepts a Stream */
typedef void (*Packet)(Stream&);

/** The packets the drone knows how to handle */
Packet packets[] = {
  // 0x00 - Ping packet used to make sure there is a connection
  [] (Stream &data) {
    drone.last_ping = 1;
  },
  // 0x01 - Prime and arm packet, echo packet
  [] (Stream &data) {
    static boolean init = false;
    if (!init) {
      servos[FR_ESC].write(1);
      servos[FL_ESC].write(1);
      servos[BR_ESC].write(1);
      servos[BL_ESC].write(1);
      scheduler.repeat(control, 4, MILLIS);
    }
  },
  // 0x02 - Send the pos to the claw
  [] (Stream &data) {
    #ifdef DEBUG
    int pos = data.parseInt();
    data.println("Claw Position: " + String(pos));
    claw.write(pos);
    #else
    claw.write(data.parseInt());
    #endif
  },
  // 0x03 - Send data to all escs
  [] (Stream &data) {
    drone.throttle = data.parseInt();
    drone.roll = data.parseInt();
    drone.pitch = data.parseInt();
    drone.yaw = data.parseInt();
  },
  // 0x04 - Show pitch roll and yaw to the current stream
  [] (Stream &data) {
    PID pid;
    float* raw_data = pid.to_vector();
    data.print(raw_data[X]); // pitch
    data.print(",");
    data.print(raw_data[Y]); // roll
    data.print(",");
    data.println(raw_data[Z]); // yaw
    delete raw_data;
  },
  // 0x05 - Show pitch roll and yaw to the current stream
  [] (Stream &data) {
    PID pid;
    data.print(pid.pitch(drone.pitch)); // pitch
    data.print(",");
    data.print(pid.roll(drone.roll)); // roll
    data.print(",");
    data.println(pid.yaw(drone.yaw)); // yaw
  },
};

/** The number of packets in the array above */
const int PACKETS = sizeof(packets) / sizeof(Packet);

/** Will process the incomming packets once there is an event trigered */
void serialEvent() {
  byte packet = Serial.parseInt();
  if (packet >= 0 && packet < PACKETS) {
    #ifdef DEBUG
    Serial.println("Packet ID: " + String(packet));
    #endif
    packets[packet](Serial); // Lets the packet process the rest of the data
  }
  // toggle the boolean to show invalid packets
  #if defined(DEBUG) && false
  else {
    Serial.println("Not a valid packet id");
  }
  #endif  
}

