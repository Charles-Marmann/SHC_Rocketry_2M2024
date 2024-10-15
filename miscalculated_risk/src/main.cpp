#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <FastLED.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <utility/imumaths.h>

//SD Card reader stuff:
#define CS_PIN 17  //Connect Chip Select pin to GPIO 17 on the Pi Pico
//end of sd card stuff


//Aerobrake and LED pin assignments
#define NUM_LEDS 4

#define LED_PWM 22

CRGB leds[NUM_LEDS];

Servo BRAKE_PWM;
//end of pwm stuff


//Sensor assignments and sensor stuff
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX bmp;

int SEALEVELPRESSURE_HPA = 1013.25;
//end of sensor stuff

int pos = 0;

void setup() {
//Connect to serial monitor
BRAKE_PWM.attach(9);
//pinMode(BRAKE_PWM, OUTPUT);
Serial.begin(115200);
  while (!Serial) delay(10); //Wait to start communiticating until serial monitor connects
  delay(5000);
Serial.println("Connected to Flight Computer");
  Serial.println("");

//Initialize communication busses
SPI.begin();
Wire.begin();

 if (!bno.begin()) { //Display error message if not able to connect to IMU
    Serial.print("Error: No IMU found on I2C bus");
    while (1);
  }

 if (!bmp.begin_I2C()) {  //Display error message if not able to connect to Barometer, defautls to I2C mode (what we are using)
    Serial.println("Error: No Barometer found on I2C bus");
    while (1);
  }
 if (!SD.begin(CS_PIN)) { //Display error message if not able to connect to Micro SD card adapter
    Serial.println("Error: Unable to initialize SD card, no adapter found on SPI bus");
    while (1); 
  }

//Add led element
FastLED.addLeds<NEOPIXEL, LED_PWM>(leds, NUM_LEDS);  //Defaults to GRB color order

//Pressure sensor settings
bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
bmp.setOutputDataRate(BMP3_ODR_50_HZ);

//Use external crystal for better accuracy
bno.setExtCrystalUse(true);
//Wait for BNO055 Calibration, then set calib LED



}

void loop() {
  // Check calibration of bno055, 3 for each means fully calibrated
for (int i = 0; i<1; i++) {
  uint8_t system, gyro, accel, mag = 0;
 bno.getCalibration(&system, &gyro, &accel, &mag);

  // Print calibration status
  Serial.print("Calibration Status: ");
  Serial.print("System: ");
  Serial.print(system);
  Serial.print(" Gyro: ");
  Serial.print(gyro);
  Serial.print(" Accel: ");
  Serial.print(accel);
  Serial.print(" Mag: ");
  Serial.println(mag);
  //analogWrite(LED_PWM,10);
}

delay(5000);
fill_solid(leds, NUM_LEDS, CRGB::Green);  // Fill all LEDs with Green
FastLED.show(); 

for (pos = 0; pos <= 360; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    BRAKE_PWM.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 360; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    BRAKE_PWM.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

//delay(5000); 
//analogWrite(LED_PWM,200);
//delay(10000);
//analogWrite(BRAKE_PWM,200);
  
delay(5000);

//analogWrite(LED_PWM,10);


}

void saveCalibration() {
  uint8_t calData[22];
  bno.getSensorOffsets(calData); // Get calibration offsets
  for (int i = 0; i < 22; i++) {
    Serial.print(calData[i]);  // Write to EEPROM
  }
}  



/*void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}
*/