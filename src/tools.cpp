#include <Arduino.h>
#include <SPI.h>
#include "macros.h"
#include "tools.h"

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
  ledcWrite(PWM_CHANNEL, 254); // Set volume Loud 1 -> 254 Quiet
  delay(500);
  ledcWrite(PWM_CHANNEL, 0);   // Turn off the buzzer
  delay(500);
}

// SPI settings (Mode 0, 1 MHz, MSB first)
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);

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

  delayMicroseconds(1); // 100 ns delay between transactions (handled by ESP32 hardware)

  // Convert raw data to temperature °C
  int16_t tempSigned = (int16_t)(tempRaw & 0xFFFC); 
  tempSigned >>= 2; 
  temperature = tempSigned * 0.03125;

  return temperature;
}

float readNTCTemperature(int pin) {
  // Read ADC (average 10 samples for stability)
  uint32_t adcSum = 0;
  const int samples = 10;
  for (int i = 0; i < samples; i++) {
    adcSum += analogRead(pin);
    delay(1);
  }
  float adcValue = adcSum / (float)samples;

  // Convert ADC to voltage
  float voltage = (adcValue / ADC_MAX) * VCC;

  // Calculate thermistor resistance
  float R_therm = (R1 *voltage) / (VCC - voltage);

  // Calculate temperature using Beta model
  float invT = 1.0 / T0 + (1.0 / BETA) * log(R_therm / R0);
  float tempK = 1.0 / invT;
  float tempC = tempK - 273.15;

  // Debug: Print resistance
  Serial.print("NTC Pin ");
  Serial.print(pin);
  Serial.print(" Resistance: ");
  Serial.print(R_therm, 0);
  Serial.println(" Ω");

  return tempC;
}
