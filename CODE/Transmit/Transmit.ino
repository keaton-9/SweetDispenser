#include <Wire.h>
int SendPin = 12;

void setup() {
  Serial.begin(9600);  // start serial for output
  

  pinMode(SendPin, OUTPUT);
}

void loop() {

  digitalWrite(SendPin, HIGH);
  delay (1000);
   digitalWrite(SendPin, LOW);
  delay (15000); //change to robot return signal???
}
