#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 9
#define LED_COUNT 4

// put function declarations here:
int flightState = 0; // 0 = prelaunch, 1 = ascent, 2 = braking, 3 = landed

// Need to verify I2C addresses
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Pin for SD reader chip select
const int chipSelect = 17;
File flightData;

void setup() {
  // put your setup code here, to run once:
  // int flightState = 1; // launch detected!
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
