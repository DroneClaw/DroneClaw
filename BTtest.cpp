#include <SoftwareSerial.h>

char val;
int ledpin = 8;

SoftwareSerial p(2,3);

void setup() {
  // put your setup code here, to run once:
  pinMode(ledpin,OUTPUT);
  //pinMode(7, OUTPUT);
  //digitalWrite(7, HIGH);
  p.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(p.available()) {
    val = p.read();
  }
  if(val == 'U') {
    digitalWrite(ledpin,HIGH);
    p.write("Hi\n");
  } else { digitalWrite(ledpin,LOW); }
  delay(100);
}
