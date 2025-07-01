#include <Arduino.h>
#include "pinout.h"

// put function declarations here:
void blink_redLed();
void blink_greenLed();
void ring_buzzer();

void setup() {
  // put your setup code here, to run once:
  pinMode(REDLED_PIN, OUTPUT);
  pinMode(GREENLED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Set up PWM for the buzzer
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION); // Configure PWM channel
  ledcAttachPin(BUZZER_PIN, PWM_CHANNEL);              // Attach buzzer pin to PWM channel

}

void loop() {
  // put your main code here, to run repeatedly:
  blink_greenLed();
  blink_redLed();
  ring_buzzer();
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

void ring_buzzer() {
  ledcWrite(PWM_CHANNEL, 128); // Set duty cycle to 50% (128 out of 255)
  delay(500);
  ledcWrite(PWM_CHANNEL, 0);   // Turn off the buzzer
  delay(500);
}