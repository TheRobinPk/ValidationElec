#ifndef _PINOUT_H
#define _PINOUT_H

#define BUZZER_PIN 13
#define GREENLED_PIN 14
#define REDLED_PIN 15
#define NTC1_PIN 25
#define NTC2_PIN 26

#define MOSI 23
#define MISO 19
#define CS 5

#define SCK 18
#define SDA 21
#define SCL 22



// PWM settings for BUZZER_PIN 
#define PWM_CHANNEL 0     // LEDC channel (0-15 available on ESP32)
#define PWM_FREQ 1000     // Frequency in Hz (1000 Hz for a typical buzzer tone)
#define PWM_RESOLUTION 8  // 8-bit resolution (0-255 duty cycle)



#endif //_PINOUT_H