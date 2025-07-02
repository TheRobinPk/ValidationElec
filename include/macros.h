#ifndef _MACROS_H
#define _MACROS_H

#define BUZZER_PIN 13
#define GREENLED_PIN 14
#define REDLED_PIN 15
#define NTC1_PIN 25
#define NTC2_PIN 26

// SPI pins
#define CS_PIN 5
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

// PWM settings for BUZZER_PIN 
#define PWM_CHANNEL 0     // LEDC channel (0-15 available on ESP32)
#define PWM_FREQ 1000     // Frequency in Hz (1000 Hz for a typical buzzer tone)
#define PWM_RESOLUTION 8  // 8-bit resolution (0-255 duty cycle)

// NTC Thermistors (NB12K00103JBB)
#define R1 10000.0  // 10 kΩ fixed resistor
#define R0 10000.0  // Thermistor resistance at 25°C
#define T0 298.15   // Reference temperature (25°C in Kelvin)
#define BETA 3977   // Beta value (B25/85)
#define VCC 3.3     // ESP32 supply voltage
#define ADC_MAX 4095 // 12-bit ADC

// TMP126 register addresses
#define TEMP_RESULT_REG 0x00 // Temperature result register

// BLE UUIDs
#define SERVICE_UUID        "abcd1234-5678-1234-5678-123456789abb" // Custom service UUID (replace if needed)
#define CHARACTERISTIC_UUID_RX "abcd1234-5678-1234-5678-123456789abc" // Write
#define CHARACTERISTIC_UUID_TX "abcd1234-5678-1234-5678-123456789abd" // Notify

#endif // _MACROS_H