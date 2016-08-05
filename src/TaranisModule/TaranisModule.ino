/*
    DroneClaw copyright 2016
*/
#include <EEPROM.h>
#include <EventLoop.h>
#include <SoftwareSerial.h>

#define PIN 2
#define CHANNELS 4
#define VERSION 0

SoftwareSerial stream(10, 11);
EventLoop scheduler;

struct {
  // Channel A - throttle
  unsigned long a_min = 187;
  unsigned long a_mid = 757;
  unsigned long a_max = 1211;
  // Channel B - pitch
  unsigned long b_min = 187;
  unsigned long b_mid = 699;
  unsigned long b_max = 1211;
  // Channel C - roll
  unsigned long c_min = 187;
  unsigned long c_mid = 698;
  unsigned long c_max = 1211;
  // Channel D - yaw
  unsigned long d_min = 189;
  unsigned long d_mid = 702;
  unsigned long d_max = 1211;
} config_eeprom;

void setup() {
  stream.begin(115200);
  Serial.begin(9600);
  pinMode(PIN, INPUT);
  //EEPROM.get(sizeof(byte), config_eeprom);
  // If eprom does not match recalibrate
  if (EEPROM.read(0) != VERSION) {
    while (!Serial);
    Serial.println("Get ready...");
    for (int i = 0 ; i < 1000 ; i++) {
      unsigned long* tmp = read_channels(); // normlize the values
      delete tmp;
    }
    delay_message(10);
    Serial.println("Old values");
    dump_eeprom();
    calibrate_eeprom();
    Serial.println("New values");
    dump_eeprom();
    EEPROM.write(0, VERSION);
  }
  // Connect the bluetooth to the drone
  stream.print("$$$");
  delay(1000);
  stream.print("C,201602225787\r");
  delay(5000);
  stream.println(0);
  delay(1000);
  stream.println(1);
  // send packets
  scheduler.repeat(read_ppm, 50, MILLIS); // ppm frame
  scheduler.repeat(ping_packet, 750, MILLIS);
  scheduler.repeat(data_packet, 500, MILLIS);
}

/** The ping packet */
void ping_packet() {
  stream.println(0);
}

void read_ppm() {
  unsigned long* tmp = read_channels();
  delete tmp;
}

/** The data packet */
void data_packet() {
  unsigned long* channels = mapped_channels();
  stream.print(3);
  stream.print(" ");
  stream.print(channels[0]);
  stream.print(" ");
  stream.print(channels[1]);
  stream.print(" ");
  stream.print(channels[2]);
  stream.print(" ");
  stream.println(channels[3]);
  delete channels;
}

void loop() {
  scheduler.process();
}

/** Print current eeprom values to screen */
void dump_eeprom() {
  Serial.println("Channel A");
  Serial.println(config_eeprom.a_min);
  Serial.println(config_eeprom.a_mid);
  Serial.println(config_eeprom.a_max);
  Serial.println("Channel B");
  Serial.println(config_eeprom.b_min);
  Serial.println(config_eeprom.b_mid);
  Serial.println(config_eeprom.b_max);
  Serial.println("Channel C");
  Serial.println(config_eeprom.c_min);
  Serial.println(config_eeprom.c_mid);
  Serial.println(config_eeprom.c_max);
  Serial.println("Channel D");
  Serial.println(config_eeprom.d_min);
  Serial.println(config_eeprom.d_mid);
  Serial.println(config_eeprom.d_max);
}

/** Calibrate and write to eeprom */
void calibrate_eeprom() {
  Serial.println("Center all sticks, with in 5 seconds");
  delay_message(5);
  unsigned long* mids = read_channels();
  Serial.println("Move all stick in round circles, for 10 seconds");
  unsigned long* min_max = calibrate_sticks(10);
  Serial.println("Done");
  config_eeprom.a_mid = mids[0];
  config_eeprom.b_mid = mids[1];
  config_eeprom.c_mid = mids[2];
  config_eeprom.d_mid = mids[3];
  config_eeprom.a_min = min_max[0];
  config_eeprom.a_max = min_max[1];
  config_eeprom.b_min = min_max[2];
  config_eeprom.b_max = min_max[3];
  config_eeprom.c_min = min_max[4];
  config_eeprom.c_max = min_max[5];
  config_eeprom.d_min = min_max[6];
  config_eeprom.d_max = min_max[7];
  EEPROM.put(sizeof(byte), config_eeprom);
  delete mids;
  delete min_max;
}

/** Delay and print the message */
void delay_message(int delta) {
  while (delta > 0) {
    Serial.print(String(delta--) + ", ");
    delay(1000);
  }
  Serial.println("done.");
}

/** Remap the channels to match what we need */
unsigned long* mapped_channels() {
  unsigned long* channels = read_channels();
  channels[0] = map(channels[0], config_eeprom.a_min, config_eeprom.a_max, 950, 1800);
  channels[1] = map(channels[1], config_eeprom.b_min, config_eeprom.b_max, -180, 180);
  channels[2] = map(channels[2], config_eeprom.c_min, config_eeprom.c_max, -180, 180);
  channels[3] = map(channels[3], config_eeprom.d_min, config_eeprom.d_max, -180, 180);
  //channels[4] = map(channels[4], config_eeprom.d_min, config_eeprom.d_max, 45, 120);
  return channels;
}

/** Read the channels and return the array */
unsigned long* read_channels() {
  static unsigned long last_channel[CHANNELS] = {};
  unsigned long* channels = new unsigned long[CHANNELS];
  for (unsigned int i = 0 ; i < CHANNELS ; i++) {
    last_channel[i] = channels[i] = 0.2 * last_channel[i] +  0.8 * pulseIn(PIN, HIGH);
  }
  return channels ;
}

/** Run the calibration and return the values min and max for each channel*/
unsigned long* calibrate_sticks(int delta) {
  unsigned long stop_time = millis() + delta * 1000; // convert to millis from seconds
  int mins[CHANNELS] = {}, maxs[CHANNELS] = {};
  unsigned long* channels = read_channels();
  // init the channels with something
  for (unsigned int i = 0 ; i < CHANNELS ; i++) {
    mins[i] = channels[i];
    maxs[i] = channels[i];
  }
  // find the mins, mids, and maxs
  do {
    Serial.println(".");
    unsigned long* channels = read_channels();
    for (unsigned int i = 0 ; i < CHANNELS ; i++) {
      if (channels[i] < mins[i]) {
        mins[i] = channels[i];
      }
      if (channels[i] > maxs[i]) {
        maxs[i] = channels[i];
      }
    }
    delete channels;
  } while (millis() < stop_time);
  unsigned long* values = new unsigned long[CHANNELS * 2];
  unsigned int k = 0;
  for (unsigned int i = 0; i < CHANNELS; i++) {
    values[i++] = mins[k];
    values[i] = maxs[k++];
  }
  return values;
}

