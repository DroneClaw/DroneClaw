char val;
int ledpin = 8;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledpin,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    val = Serial.read();
  }
  if(val == 'H') {
    digitalWrite(ledpin,HIGH);
    Serial.write("Hello World\n");
  } else { digitalWrite(ledpin,LOW); }
  delay(100);
}
