#include <Arduino.h>
#include "pinout.h"

// put function declarations here:

void setup() {
  // put your setup code here, to run once:
  pinMode(REDLED_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(REDLED_PIN, HIGH);
  delay(500);
  digitalWrite(REDLED_PIN, LOW);
  delay(500);  
}
