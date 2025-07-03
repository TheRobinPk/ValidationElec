#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_INA237.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>
#include "macros.h"
#include "tools.h"

// INA237 instance
Adafruit_INA237 ina237 = Adafruit_INA237();

bool is_INA_connected = false;

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic = NULL;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Disconnected");
    BLEDevice::startAdvertising();
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0 && deviceConnected) {
      String command = String(rxValue.c_str());
      Serial.println("Received: " + command);
      if (command.equalsIgnoreCase("LED")) {
        Serial.println("Blinking LED");
        for (int i = 0; i < 3; i++) {
          blink_greenLed(); // Blink green LED
          blink_redLed();   // Blink red LED
        }
        pTxCharacteristic->setValue("LED Blinked");
        pTxCharacteristic->notify();
        Serial.println("Sent: LED Blinked");
      }
      else if (command.equalsIgnoreCase("BUZZER")) {
        Serial.println("Buzzer Ringing");
        for (int i = 0; i < 3; i++) {
          ring_buzzer();
        }
        pTxCharacteristic->setValue("Buzzer Ringed");
        pTxCharacteristic->notify();
      }
      else if (command.equalsIgnoreCase("SPI"))
      {
        Serial.println("Reading SPI Temperature");
        float temp = readTemperature();
        Serial.print("Temperature: ");
        Serial.print(temp, 2); 
        Serial.println(" °C");
        pTxCharacteristic->setValue("SPI done");
        pTxCharacteristic->notify();
        Serial.println("Sent: SPI done");

      }
      else if (is_INA_connected && (command.equalsIgnoreCase("I2C"))||(command.equalsIgnoreCase("INA"))) // TODO : separate INA from I2C
      {
        Serial.println("Reading I2C Information");
        Serial.printf("Current: %.2f mA, Bus Voltage: %.2f V, Shunt Voltage: %.0f uV, Power: %.2f mW, Temp: %.2f C\n",
              ina237.getCurrent_mA(),
              ina237.getBusVoltage_V(),
              ina237.getShuntVoltage_mV() * 1000.0, // Convert mV to μV
              ina237.getPower_mW(),
              ina237.readDieTemp());
        pTxCharacteristic->setValue("I2C done");
        pTxCharacteristic->notify();
        Serial.println("Sent: I2C done");

      }
      else if (command.equalsIgnoreCase("CTN"))
      {
        Serial.println("Reading CTN Temperature");
        float ntc1Temp = readNTCTemperature(NTC1_PIN);
        float ntc2Temp = readNTCTemperature(NTC2_PIN);
        Serial.printf("NTC1 (GPIO26) Temp: %.2f C, NTC2 (GPIO25) Temp: %.2f C\n", ntc1Temp, ntc2Temp);
        pTxCharacteristic->setValue("I2C done");
        pTxCharacteristic->notify();
        Serial.println("Sent: I2C done");

      }
    }
  }
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000); 
  
  pinMode(REDLED_PIN, OUTPUT);
  pinMode(GREENLED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(GREENLED_PIN, LOW);
  digitalWrite(REDLED_PIN, LOW);
  
  // Set up PWM for the buzzer
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION); // Configure PWM channel
  ledcAttachPin(BUZZER_PIN, PWM_CHANNEL);           // Attach buzzer pin to PWM channel
  ledcWrite(PWM_CHANNEL, 0);

  //setup SPI communication
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Deselect chip
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  // Initialize the INA237 sensor
  if (!ina237.begin()) {
    Serial.println("Couldn't find INA237 chip");
    is_INA_connected = false;
  } else {
    // Configure shunt resistance (0.5 mΩ) and max current (10 A)
    ina237.setShunt(0.0005, 10.0);
    // Set averaging and conversion times for better accuracy
    ina237.setAveragingCount(INA2XX_COUNT_16);
    ina237.setVoltageConversionTime(INA2XX_TIME_150_us);
    ina237.setCurrentConversionTime(INA2XX_TIME_280_us);
    is_INA_connected = true;
  }

  // Initialize ADC
  analogReadResolution(12); // 12-bit resolution
  analogSetAttenuation(ADC_11db); // 0–3.3V range


  BLEDevice::init("ESP32_RAA");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pRxCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();
  BLEDevice::getAdvertising()->start();
  Serial.println("BLE started as ESP32_RAA");
}

void loop() {
  delay(1000); // Keep loop minimal
}
