

/***
  Read Our Complete Guide: https://RandomNerdTutorials.com/esp32-bme680-sensor-arduino/
  Designed specifically to work with the Adafruit BME680 Breakout ----> http://www.adafruit.com/products/3660 These sensors use I2C or SPI to communicate, 2 or 4 pins are required to interface. Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit! Written by Limor Fried & Kevin Townsend for Adafruit Industries. BSD license, all text above must be included in any redistribution
***/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <HardwareSerial.h>

#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15



uint32_t secretNumber; // Replace this with your 10-digit secret number
bool gameActive = true;
bool gameActive1 = true;

#define SEALEVELPRESSURE_HPA (1013.25)

//Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
float T, H, P, G;
const int ledPin = 0;

void setup() {
    pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
  delay(200);
  digitalWrite(ledPin, HIGH);

  Serial.begin(115200);
  pinMode(2, OUTPUT);
  while (!Serial);
  Serial.println("start");
  Serial.println(F("BME688 async test"));
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME688 sensor, check wiring!"));
    Serial.println("2525");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms



}

void loop() {
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }

  delay(50); // This represents parallel work.

  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
 
  T = bme.temperature;
  int t = int(T);
  Serial.println(t);
  H = bme.humidity ;
  int h = int(H);
  Serial.println(h);
  P = bme.pressure / 100.0;
  int p = int(P);
  Serial.println(p);
  G = bme.gas_resistance / 1000.0;
  int g = int(G);
  Serial.println(g);

  // Send the data over UART to the Catre Lora-E5
 // Serial.println(dataToSend);
    if (gameActive) {
        // Check if data is available on UART (Serial Monitor)
        if (Serial.available() > 0) {
            // Read the received data from Serial Monitor
            uint32_t receivedNumber = Serial.parseInt();
            Serial.println(receivedNumber);

            // Check if the received number is the first guess
            if (receivedNumber == 800 || receivedNumber == 30 || receivedNumber == 10 || receivedNumber == 3 ) {
                // Update the secret number based on the first guess
                if (receivedNumber == 800) {
                    secretNumber = p;
                    gameActive1 = true;
                } else if (receivedNumber == 30) {
                    secretNumber = g;
                    gameActive1 = true;
                } else if (receivedNumber == 10) {
                    secretNumber = h;
                    gameActive1 = true;
                } else if (receivedNumber == 3) {
                    secretNumber = t;
                    gameActive1 = true;
                }
            }
            
           while (gameActive1 ) {
             Serial.println(secretNumber);
             if (Serial.available() > 0) {
            // Read the received data from Serial Monitor
            uint32_t receivedNumber = Serial.parseInt();
            Serial.println(receivedNumber);
            // Compare the received number with the secret number
            if (receivedNumber == secretNumber) {
                // Correct guess, set pin 16 to HIGH (connected to STM32WL)
                digitalWrite(2, HIGH);
                delay(200);
                
                gameActive1 = false; // End the game, prevent further guesses
                Serial.println("Correct Guess! Game Over.");
                digitalWrite(2, LOW);
            } else {
                // Incorrect guess, continue the game
                Serial.println("Incorrect Guess! Try again.");
            }
            }
           }

        }
    }
  delay(200);
}




