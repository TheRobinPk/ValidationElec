#include <Arduino.h>
#include "pinout.h"

// put function declarations here:
void blink_redLed();
void blink_greenLed();

void setup() {
  // put your setup code here, to run once:
  pinMode(REDLED_PIN, OUTPUT);
  pinMode(GREENLED_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  blink_greenLed();
  blink_redLed();
}

void blink_redLed() {
  digitalWrite(REDLED_PIN, HIGH);
  delay(500);
  digitalWrite(REDLED_PIN, LOW);
  delay(500);
}

void blink_greenLed() {
  digitalWrite(GREENLED_PIN, HIGH);
  delay(500);
  digitalWrite(GREENLED_PIN, LOW);
  delay(500);
}