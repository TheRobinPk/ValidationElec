#include <Arduino.h>
#include <SPI.h>
#include "pinout.h"


// put function declarations here:
void blink_redLed();
void blink_greenLed();
void ring_buzzer();
float readTemperature();

// TMP126 register addresses
#define TEMP_RESULT_REG 0x00 // Temperature result register
// SPI settings (Mode 0, 1 MHz, MSB first)
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);


void setup() {
  // put your setup code here, to run once:
  pinMode(REDLED_PIN, OUTPUT);
  pinMode(GREENLED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Set up PWM for the buzzer
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION); // Configure PWM channel
  ledcAttachPin(BUZZER_PIN, PWM_CHANNEL);              // Attach buzzer pin to PWM channel

  //setup SPI communication
  Serial.begin(115200);
  delay(1000); 

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Deselect chip

  // Initialize SPI
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

}

void loop() {
  // put your main code here, to run repeatedly:
  blink_greenLed();
  blink_redLed();
  ring_buzzer();
    
  float temp = readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temp, 2); 
  Serial.println(" °C");



   
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

float readTemperature() {
  uint16_t tempRaw;
  float temperature;


  uint16_t command = (1 << 8) | TEMP_RESULT_REG; 

  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);


  SPI.transfer16(command);

  // Read 16-bit temperature data
  tempRaw = SPI.transfer16(0x0000); // Send dummy data to read response

  digitalWrite(CS_PIN, HIGH); 
  SPI.endTransaction();

  // Ensure 100 ns delay between transactions (handled by ESP32 hardware)
  delayMicroseconds(1);

  // Convert raw data to temperature °C
  int16_t tempSigned = (int16_t)(tempRaw & 0xFFFC); 
  tempSigned >>= 2; 
  temperature = tempSigned * 0.03125;

  return temperature;
}

