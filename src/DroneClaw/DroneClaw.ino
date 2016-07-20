/*
    DroneClaw copyright 2016
*/
#include <EventLoop.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// Will enable debug code throught the program
//#define DEBUG

// Escs
#define FL_ESC 5
#define FR_ESC 6
#define BL_ESC 10
#define BR_ESC 11

#define CLAW 3

#define BT_RX 8
#define BT_TX 9

void println(String);
void attach();
void working();

EventLoop &scheduler = EventLoop::get();

// The main instance of the drone claw used to pass around data with
struct Drone {
    static const byte SERVOS = 4;
    Servo servos[SERVOS];
    SoftwareSerial bluetooth = SoftwareSerial(BT_RX, BT_TX);
    Servo claw = Servo();
    /** Init the drone */
    inline Drone() {
      //println("Constructing the Drone...");
      for (byte i = 0; i < SERVOS; i++) {
        servos[i] = Servo();
      }
      bluetooth.begin(9600);
      claw.attach(CLAW);
    }
    static void attach();
    static void all(const byte);
    static void prime();
    static void arm();
    static void disarm();
} drone;

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
    /** Will process the incomming packets */
    static void process_packets();
};

/** The packets the drone knows how to handle */
Packet packets[] = {
  // Ping packet used to make sure there is a connection
  Packet(0x01, [] (SoftwareSerial &data) {
    // todo if last heart beat fails do something
  }),
  // Prime and arm packet, echo packet
  Packet(0x02, [] (SoftwareSerial &data) {
    Drone::prime();
    Drone::arm();
  }),
  // Send the pos to the claw
  Packet(0x03, [] (SoftwareSerial &data) {
    int pos = data.readString().toInt();
    println("Claw Position: " + String(pos));
    drone.claw.write(data.readString().toInt());
  }),
  // Send data to all escs
  Packet(0x04, [] (SoftwareSerial &data) {
    byte pos = (byte) data.readString().toInt();
    Drone::all(pos);
  }),
};

/** Will process the incomming packets */
void Packet::process_packets()  {
  if (drone.bluetooth.available() > 0) {
    byte packet = drone.bluetooth.read() - 'a'; // Get the packet id, treat a = 0, b = 2 ...
    if (packet >= 0) {
      println("Packet ID: " + String(packet));
      packets[packet].decode(drone.bluetooth); // Lets the packet process the rest of the data
    }
  }
}

/** Attach all the esc to the servo instances */
void Drone::attach() {
  static const byte pwms[] = {FL_ESC, FR_ESC, BL_ESC, BR_ESC};
  static byte tmp = 0;
  for (byte i = 0; i < Drone::SERVOS; i++) {
    scheduler.execute([] () {
      println("Attaching esc " + String(tmp + 1) + " to " + String(pwms[tmp]));
      drone.servos[tmp++].attach(pwms[tmp]);
    });
  }
  // Reset tmp back to 0
  scheduler.execute([] () {
    tmp = 0;
  });
}

/** Send a value to all the motors */
void Drone::all(const byte number) {
  static struct _ {
    byte number;
  } _;
  _.number = number;
  scheduler.execute([] () {
    for (byte i = 0; i < Drone::SERVOS; i++) {
      println("Executing esc " + String(i + 1) + " with " + String(_.number));
      drone.servos[i].write(_.number);
      delay(5);
    }
  });
}

/** Prime the esc's by sending value 1 */
void Drone::prime() {
  println("Priming...");
  all(1);
}

/** Arm the motors spining at lowest speed */
void Drone::arm() {
  println("Arming...");
  all(50);
}

/** Stop all the motors */
void Drone::disarm() {
  println("Disarming...");
  all(0);
}

/** Print a message to the Serial only when debug is defined */
void println(String msg) {
  #ifdef DEBUG
  static boolean begin = true;
  if (begin) {
    Serial.begin(9600);
    begin = false;
  }
  Serial.println(msg);
  #endif
  drone.bluetooth.println(msg);
}


/** Simple working led */
void working() {
  const byte LED = 13;
  pinMode(LED, OUTPUT);
  // Turn on the LED every 2 secs
  scheduler.repeat([] () {
    digitalWrite(LED, HIGH);
    // Turn off the LED off after 1 sec
    scheduler.execute([] () {
      digitalWrite(LED, LOW);
    }, 1, SECONDS);
  }, 2, SECONDS);
}

void setup() {
  println("DroneClaw");
  working();
  Drone::attach();
  scheduler.repeat(Packet::process_packets);
}

void loop() {
    scheduler.process(); // The backbone of the system
}
