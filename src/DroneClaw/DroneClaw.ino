/*
    DroneClaw copyright 2016
*/
#include <EventLoop.h>
#include <Servo.h>

// Will enable debug code throught the program
#define DEBUG

void println(String);
void attach();
void all();
void prime();
void arm();
void disarm();
void working();

EventLoop &scheduler = EventLoop::get();

// The main instance of the drone claw used to pass around data with
struct Drone {
    static const byte SERVOS = 4;
    Servo servos[SERVOS];
    boolean primed = false;
    boolean armed = false;
    /** Init the drone */
    inline Drone() {
      //println("Constructing the Drone...");
      for (byte i = 0; i < SERVOS; i++) {
        servos[i] = Servo();
      }
    }
} drone;

/** Attach all the esc to the servo instances */
void attach() {
  static const byte pwms[] = {5, 6, 10, 11};
  static byte tmp = 0;
  for (byte i = 0; i < Drone::SERVOS; i++) {
    EventLoop::get().execute([] () {
      println("Attaching esc " + String(tmp + 1) + " to " + String(pwms[tmp]));
      drone.servos[tmp++].attach(pwms[tmp]);
    });
  }
  // Reset tmp back to 0
  EventLoop::get().execute([] () {
    tmp = 0;
  });
}

/** Send a value to all the motors */
void all(const byte number) {
  static struct _ {
    byte number;
  } _;
  _.number = number;
  scheduler.execute([] () {
    for (byte i = 0; i < Drone::SERVOS; i++) {
      println("Executing esc " + String(i + 1) + " with " + String(_.number));
      drone.servos[i].write(_.number);
    }
  });
}

/** Prime the esc's by sending value 1 */
void prime() {
  println("Priming...");
  all(1);
  drone.primed = true;
}

/** Arm the motors spining at lowest speed */
void arm() {
  println("Arming...");
  all(50);
  drone.armed = true;
}

/** Stop all the motors */
void disarm() {
  println("Disarming...");
  all(0);
  drone.armed = false;
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
}

void setup() {
  println("DroneClaw");
  console();
  working();
  attach();
  scheduler.execute(prime, 1, SECONDS);
  scheduler.execute(arm, 5, SECONDS);
  // Test the flight
  scheduler.repeat([] () {
    if (drone.primed && drone.armed) {
      int mapped = map(analogRead(A0), 200, 1024, 0, 130) + 50;
      int value = mapped < 50 ? 0 : mapped;
      if (value > 0) {
        all(value);
      }
    }
  }, 50, MILLIS);
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

/** Console the manage debug commands */
void console() {
  #ifdef DEBUG
  scheduler.repeat([] () {
    if (Serial.available()) {
      String value = Serial.readString();
      String cmd = value.substring(0, value.indexOf(' '));
      if (value[0] != '/') return; // not a command
      println("Processing command: " + cmd);
      if (cmd == "/prime") { // Prime the esc's
        prime();
      } else if (cmd == "/attach") { // Attach the esc's
        attach();
      } else if (cmd == "/arm") { // Arm the drone
        arm();
      } else if (cmd == "/disarm" || cmd == "//") { // Disarm the drone
        disarm();
      } else if (cmd == "/speed") { // Change the speed of the motors
        int speed = value.substring(value.indexOf(' '), value.length()).toInt();
        all(speed);
      }
    }
  }, 20, MILLIS);
  #endif
}

void loop() {
    scheduler.process(); // The backbone of the system
}
