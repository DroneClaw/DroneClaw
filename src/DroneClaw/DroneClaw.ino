/*
    DroneClaw copyright 2016
*/
#include <EventLoop.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>

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

#ifdef DEBUG
void println(String);
#endif
void attach();
void all(byte);
void working();

EventLoop &scheduler = EventLoop::get();

const byte pwms[] = {FL_ESC, FR_ESC, BL_ESC, BR_ESC};

Servo servos[SERVOS];
SoftwareSerial bluetooth(BT_RX, BT_TX);
Servo claw;


/** The data from the MPU in a struct form */
class MPU {
  #define ADDRESS 0x68
  #define START_ADDRESS 0x3b
  public:
    int accel_x, accel_y, accel_z;
    int gyro_x, gyro_y, gyro_z;
    int temp;
    /** Create the instance of this MPU struct with the values from the sensor */
    inline MPU() {
      Wire.beginTransmission(ADDRESS);
      Wire.write(START_ADDRESS);
      Wire.endTransmission();
      Wire.requestFrom(ADDRESS, 14);
      gyro_x = Wire.read() << 8 | Wire.read();
      gyro_y = Wire.read() << 8 | Wire.read();
      gyro_z = Wire.read() << 8 | Wire.read();
      temp = Wire.read() << 8 | Wire.read();
      accel_x = Wire.read() << 8 | Wire.read();
      accel_y = Wire.read() << 8 | Wire.read();
      accel_z = Wire.read() << 8 | Wire.read();
    }
    /** Get the raw output from the MPU */
    inline int* raw_output() {
      int raw[7];
      raw[0] = accel_x;
      raw[1] = accel_y;
      raw[2] = accel_z;
      raw[3] = temp;
      raw[4] = gyro_x;
      raw[5] = gyro_y;
      raw[6] = gyro_z;
      return raw;
    }
    /** Inits the Wire lib and the sensors */
    inline static void init() {
      Wire.begin();
      Wire.beginTransmission(ADDRESS);
      Wire.write(0x6B);
      Wire.write(0);
      Wire.endTransmission();
      Wire.beginTransmission(ADDRESS);
      Wire.write(0x1C);
      Wire.write(0x10);
      Wire.endTransmission();
      Wire.beginTransmission(ADDRESS);
      Wire.write(0x1B);
      Wire.write(0x08);
      Wire.endTransmission();
    }
};

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
    #ifdef DEBUG
    println("Claw Position: " + String(pos));
    #endif
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
      #ifdef DEBUG
      println("Packet ID: " + String(packet));
      #endif
      packets[packet].decode(bluetooth); // Lets the packet process the rest of the data
    } else {
      #ifdef DEBUG
      println("Not a valid packet id");
      #endif
    }
  }
}

/** Send a value to all the motors */
void all(const byte number) {
  for (byte i = 0; i < SERVOS; i++) {
    #ifdef DEBUG
    println("Executing esc " + String(i + 1) + " with " + String(number));
    #endif
    servos[i].write(number);
  }
}

#ifdef DEBUG
/** Print a message to the Serial only when debug is defined */
void println(String msg) {
  static boolean begin = true;
  if (begin) {
    Serial.begin(BUAD);
    begin = false;
  }
  Serial.println(msg);
  bluetooth.println(msg);
}
#endif

void setup() {
  // Get the sensors and ect ready before bluetooth connection is established
  bluetooth.begin(BUAD);
  claw.attach(CLAW);
  MPU::init();
  
  // Make sure the connection is established
  byte pos = 45;
  byte flip = 1; 
  while(!bluetooth.available() || bluetooth.parseInt() != 0) {
    if (pos > 135 || pos < 45) {
      flip *= -1;
      delay(250);
    }
    pos += flip;
    claw.write(pos);
    delay(10);
  }
  claw.write(45);

  #ifdef DEBUG
  println("\n----- DroneClaw -----\n");
  #endif

  // Set up escs
  for (byte i = 0; i < SERVOS; i++) {
    servos[i] = Servo();
    #ifdef DEBUG
    println("Attaching esc " + String(i) + " to " + String(pwms[i]));
    #endif
    servos[i].attach(pwms[i]);
  }
  
  // Handle the packets
  scheduler.repeat(process_packets);
}

void loop() {
  scheduler.process(); // The backbone of the system
}
