#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_INA237.h>
#include <Arduino.h>
#include <math.h>
#include "pinout.h"

// put function declarations here:
void blink_redLed();
void blink_greenLed();
void ring_buzzer();
float readTemperature();
float readNTCTemperature(int pin);

// TMP126 register addresses
#define TEMP_RESULT_REG 0x00 // Temperature result register
// SPI settings (Mode 0, 1 MHz, MSB first)
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
// INA237 instance
Adafruit_INA237 ina237 = Adafruit_INA237();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000); 
  
  pinMode(REDLED_PIN, OUTPUT);
  pinMode(GREENLED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Set up PWM for the buzzer
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION); // Configure PWM channel
  ledcAttachPin(BUZZER_PIN, PWM_CHANNEL);              // Attach buzzer pin to PWM channel

  //setup SPI communication
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Deselect chip
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

// Initialize the INA237 sensor
  if (!ina237.begin()) {
    Serial.println("Couldn't find INA237 chip");
    while (1);
  }

  // Configure shunt resistance (15 mΩ) and max current (10 A)
  ina237.setShunt(0.015, 10.0);
  // Set averaging and conversion times for better accuracy
  ina237.setAveragingCount(INA2XX_COUNT_16);
  ina237.setVoltageConversionTime(INA2XX_TIME_150_us);
  ina237.setCurrentConversionTime(INA2XX_TIME_280_us);

  // Initialize ADC
  analogReadResolution(12); // 12-bit resolution
  analogSetAttenuation(ADC_11db); // 0–3.3V range

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

  Serial.printf("Current: %.2f mA, Bus Voltage: %.2f V, Shunt Voltage: %.0f uV, Power: %.2f mW, Temp: %.2f C\n",
                ina237.getCurrent_mA(),
                ina237.getBusVoltage_V(),
                ina237.getShuntVoltage_mV() * 1000.0, // Convert mV to μV
                ina237.getPower_mW(),
                ina237.readDieTemp());

  float ntc1Temp = readNTCTemperature(NTC1_PIN);
  float ntc2Temp = readNTCTemperature(NTC2_PIN);
  Serial.printf("NTC1 (GPIO26) Temp: %.2f C, NTC2 (GPIO25) Temp: %.2f C\n", ntc1Temp, ntc2Temp);
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
  ledcWrite(PWM_CHANNEL, 254); // Set volume Loud 1 -> 254 Quiet
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
  Serial.print("NTC Pin "); Serial.print(pin); Serial.print(" Resistance: "); Serial.print(R_therm, 0); Serial.println(" Ω");

  return tempC;
}
