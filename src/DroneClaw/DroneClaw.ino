/*
    DroneClaw copyright 2016
*/
#include <EventLoop.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Will enable debug code throught the program
//#define DEBUG

#define SERVOS 4
#define FL_ESC 5
#define FR_ESC 6
#define BL_ESC 10
#define BR_ESC 11
#define CLAW 3
#define BT_RX 8
#define BT_TX 9
#define PACKETS 4
#define BAUD 9600

#ifdef DEBUG
void println(String);
#endif
void attach();
void all(byte);
void working();
void control();

EventLoop &scheduler = EventLoop::get();

const byte pwms[] = {FL_ESC, FR_ESC, BL_ESC, BR_ESC};

struct data {
  int throttle = 0;
  int pitch = 0;
  int roll = 0;
} drone;

int offset[3] = {0, 0, 0}; // x, y, z
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
      while (Wire.available() < 14);
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
    void (*_function)(Stream&);
  public:
    inline Packet(byte id, void (*function)(Stream&)) {
      _id = id;
      _function = function;
    }
    /** This will decode the data */
    inline void decode(Stream &data) {
      _function(data);
    }
};

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
      all(1);
      scheduler.repeat(control);
    }
  }),
  // Send the pos to the claw
  Packet(0x02, [] (Stream &data) {
    int pos = data.parseInt();
    #ifdef DEBUG
    println("Claw Position: " + String(pos));
    #endif
    claw.write(pos);
  }),
  // Send data to all escs
  Packet(0x03, [] (Stream &data) {
    int pos = data.parseInt();
    //all(pos);
    drone.throttle = pos;

    //drone.pitch = data.parseInt();
    //drone.roll = data.parseInt();
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
  } else {
    #ifdef DEBUG
    //println("Not a valid packet id");
    #endif
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

/** The main control loop */
void control() {
  int throttle = drone.throttle;
  
  if (throttle < 1000) return;
  
  MPU mpu;
  int pitch = map(mpu.gyro_y - offset[1], -4096, 4096, -90, 90);
  int roll = map(mpu.gyro_x - offset[0], -4096, 4096, -90, 90);
  pitch *= 2;
  roll *= 2;
  int fl = throttle, fr = throttle, bl = throttle, br = throttle;
  
  bl += pitch;
  br += pitch;
  fl -= pitch;
  fr -= pitch;
  bl -= roll;
  br += roll;
  fl -= roll;
  fr += roll;

  servos[1].writeMicroseconds(fr);
  servos[3].writeMicroseconds(br);
  servos[0].writeMicroseconds(fl);
  servos[2].writeMicroseconds(bl);
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

  // Calibrate the gyro
  #define COUNT 2000
  for (int i = 0 ; i < COUNT; i++) {
    MPU mpu;
    offset[0] += mpu.gyro_x;
    offset[1] += mpu.gyro_y;
    offset[2] += mpu.gyro_z;
  }
  offset[0] /= COUNT;
  offset[1] /= COUNT;
  offset[2] /= COUNT;

  // Make sure the connection is established
  byte pos = 45;
  byte flip = 1;
  while ((!Serial.available() && !bluetooth.available()) || (Serial.parseInt() != 0 && bluetooth.parseInt() != 0)) {
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
