#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>
#include "Adafruit_BMP3XX.h"

// put function declarations here:
int flightState = 0; // 0 = prelaunch, 1 = ascent, 2 = braking, 3 = landed

void setup() {
  // put your setup code here, to run once:
  // int flightState = 1; // launch detected!
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
