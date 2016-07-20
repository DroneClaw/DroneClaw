/*
    DroneClaw copyright 2016
*/
#include <EventLoop.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// Will enable debug code throught the program
//#define DEBUG

// Escs
#define SERVOS 4
#define FL_ESC 5
#define FR_ESC 6
#define BL_ESC 10
#define BR_ESC 11

#define CLAW 3

#define BT_RX 8
#define BT_TX 9

#define PACKETS 4

#define BUAD 9600

void println(String);
void attach();
void all(byte);
void working();

EventLoop &scheduler = EventLoop::get();

const byte pwms[] = {FL_ESC, FR_ESC, BL_ESC, BR_ESC};

Servo servos[SERVOS];
SoftwareSerial bluetooth(BT_RX, BT_TX);
Servo claw;

/** The packet class that can encode and decode data*/
class Packet {
  private:
    byte _id;
    void (*_function)(SoftwareSerial&);
  public:
    inline Packet(byte id, void (*function)(SoftwareSerial&)) {
      _id = id;
      _function = function;
    }
    /** This will decode the data */
    inline void decode(SoftwareSerial &data) {
      _function(data);
    }
};

/** The packets the drone knows how to handle */
Packet packets[] = {
  // Ping packet used to make sure there is a connection
  Packet(0x00, [] (SoftwareSerial &data) {
    // todo if last heart beat fails do something
  }),
  // Prime and arm packet, echo packet
  Packet(0x01, [] (SoftwareSerial &data) {
    all(1);
  }),
  // Send the pos to the claw
  Packet(0x02, [] (SoftwareSerial &data) {
    int pos = data.parseInt();
    println("Claw Position: " + String(pos));
    claw.write(pos);
  }),
  // Send data to all escs
  Packet(0x03, [] (SoftwareSerial &data) {
    int pos = data.parseInt();
    all(pos);
  }),
};

/** Will process the incomming packets */
void process_packets() {
  if (bluetooth.available()) {
    byte packet = bluetooth.parseInt(); // Get the packet id
    if (packet >= 0 && packet < PACKETS) {
      println("Packet ID: " + String(packet));
      packets[packet].decode(bluetooth); // Lets the packet process the rest of the data
    } else {
      println("Not a valid packet id");
    }
  }
}

/** Send a value to all the motors */
void all(const byte number) {
  for (byte i = 0; i < SERVOS; i++) {
    println("Executing esc " + String(i + 1) + " with " + String(number));
    servos[i].write(number);
    delay(5);
  }
}

/** Print a message to the Serial only when debug is defined */
void println(String msg) {
  #ifdef DEBUG
  static boolean begin = true;
  if (begin) {
    Serial.begin(BUAD);
    begin = false;
  }
  Serial.println(msg);
  #endif
  bluetooth.println(msg);
}

void setup() {
  bluetooth.begin(BUAD);

  // Make sure the connection is established
  while(!bluetooth.available() || bluetooth.parseInt() != 0);
  
  println("\n----- DroneClaw -----\n");

  // Set up escs
  for (byte i = 0; i < SERVOS; i++) {
    servos[i] = Servo();
    println("Attaching esc " + String(i) + " to " + String(pwms[i]));
    servos[i].attach(pwms[i]);
  }

  claw.attach(CLAW);
  
  // Handle the packets
  scheduler.repeat(process_packets);
}

void loop() {
  scheduler.process(); // The backbone of the system
}
