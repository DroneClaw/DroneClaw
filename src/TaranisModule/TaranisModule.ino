/*
    DroneClaw copyright 2016
*/
#include <CurieEEPROM.h>
#include <EventLoop.h>

#define PIN 2
#define CHANNELS 4
#define VERSION 0

EventLoop scheduler;

struct {
  // Channel A - throttle
  int a_min = 187;
  int a_mid = 757;
  int a_max = 1211;
  // Channel B - pitch
  int b_min = 187;
  int b_mid = 699;
  int b_max = 1211;
  // Channel C - roll
  int c_min = 187;
  int c_mid = 698;
  int c_max = 1211;
  // Channel D - yaw
  int d_min = 189;
  int d_mid = 702;
  int d_max = 1211;
} config_eeprom;

void setup() {
  Serial.begin(9600);
  pinMode(PIN, INPUT);
  //EEPROM.get(sizeof(byte), config_eeprom);
  // If eprom does not match recalibrate
  if (EEPROM.read(0) != VERSION) {
    while (!Serial);
    Serial.println("Get ready...");
    for (int i = 0 ; i < 1000 ; i++) {
      int* tmp = read_channels(); // normlize the values
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
}

void loop() {
  scheduler.process();
  int* channels = mapped_channels();
  Serial.print(channels[0]);
  Serial.print(",");
  Serial.print(channels[1]);
  Serial.print(",");
  Serial.print(channels[2]);
  Serial.print(",");
  Serial.println(channels[3]);
  delete channels;
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
  int* mids = read_channels();
  Serial.println("Move all stick in round circles, for 10 seconds");
  int* min_max = calibrate_sticks(10);
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
int* mapped_channels() {
  int* channels = read_channels();
  channels[0] = map(channels[0], config_eeprom.a_min, config_eeprom.a_max, 1000, 1800);
  channels[1] = map(channels[1], config_eeprom.b_min, config_eeprom.b_max, -180, 180);
  channels[2] = map(channels[2], config_eeprom.c_min, config_eeprom.c_max, -180, 180);
  channels[3] = map(channels[3], config_eeprom.d_min, config_eeprom.d_max, -180, 180);
  return channels;
}

/** Read the channels and return the array */
int* read_channels() {
  static int last_channel[CHANNELS] = {};
  int* channels = new int[CHANNELS];
  for (int i = 0 ; i < CHANNELS ; i++) {
    last_channel[i] = channels[i] = 0.2 * last_channel[i] +  0.8 * pulseIn(PIN, HIGH);
  }
  return channels;
}

/** Run the calibration and return the values min and max for each channel*/
int* calibrate_sticks(int delta) {
  unsigned long stop_time = millis() + delta * 1000; // convert to millis from seconds
  int mins[CHANNELS] = {}, maxs[CHANNELS] = {};
  int* channels = read_channels();
  // init the channels with something
  for (int i = 0 ; i < CHANNELS ; i++) {
    mins[i] = channels[i];
    maxs[i] = channels[i];
  }
  // find the mins, mids, and maxs
  do {
    Serial.println(".");
    int* channels = read_channels();
    for (int i = 0 ; i < CHANNELS ; i++) {
      if (channels[i] < mins[i]) {
        mins[i] = channels[i];
      }
      if (channels[i] > maxs[i]) {
        maxs[i] = channels[i];
      }
    }
    delete channels;
  } while (millis() < stop_time);
  int* values = new int[CHANNELS * 2];
  int k = 0;
  for (int i = 0; i < CHANNELS; i++) {
    values[i++] = mins[k];
    values[i] = maxs[k++];
  }
  return values;
}

