
/*
    DroneClaw copyright 2016
*/
#include <CurieEEPROM.h>

#define PIN 2

enum ConfigKeys {
  ID,
  // Channel A
  A_MIN,
  A_MID,
  A_MAX,
  // Channel B
  B_MIN,
  B_MID,
  B_MAX,
  // Channel C
  C_MIN,
  C_MID,
  C_MAX,
  // Channel D
  D_MIN,
  D_MID,
  D_MAX,
};


#define NAME 'D' ^ 'A' ^ 'T' ^ 'A' ^ '-' ^ 'V' ^ 1



void setup() {
  Serial.begin(9600);
  pinMode(PIN, INPUT);

  // If eprom does not match recalibrate
  if (EEPROM.read(ID) != NAME) {
    EEPROM.write(ID, NAME);
  }
}

int a,b,c,d;



void loop() {
  a = a * .5 + pulseIn(PIN, HIGH) * .5;
  b = b * .5 + pulseIn(PIN, HIGH) * .5;
  c = c * .5 + pulseIn(PIN, HIGH) * .5;
  d = d * .5 + pulseIn(PIN, HIGH) * .5;
  Serial.print(a);
  Serial.print(",");
  Serial.print(b);
  Serial.print(",");
  Serial.print(c);
  Serial.print(",");
  Serial.println(d);
}
