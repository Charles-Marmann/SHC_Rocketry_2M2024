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
#include <hardware/flash.h>

//using serial? turn off for flight
const bool output_serial = true;

//SD Card reader stuff:
#define CS_PIN 17  //Connect Chip Select pin to GPIO 17 on the Pi Pico
//end of sd card stuff

//offset from start of flash memory for BNO055 calibration data
//we can access this at XIP_BASE + 1024k.
#define CALIB_FLASH_OFFSET (1024 * 1024)

//Aerobrake and LED pin assignments
#define NUM_LEDS 4

#define LED_PWM 22

CRGB leds[NUM_LEDS];

Servo BRAKE_PWM;
//end of pwm stuff

//Sensor assignments and sensor stuff
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BMP3XX bmp;
int SEALEVELPRESSURE_HPA = 1013.25;
//end of sensor stuff

int pos = 0;

void setup() {
//Connect to serial monitor
BRAKE_PWM.attach(9);
//pinMode(BRAKE_PWM, OUTPUT);
if (output_serial) 
{
  Serial.begin(115200);
  while (!Serial) delay(10); //Wait to start communiticating until serial monitor connects
  delay(5000);
  Serial.println("Connected to Flight Computer");
  Serial.println("");
}
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

//Begin BNO055 setup

//Search for BNO055 calibration data
//romAddress is our current place in ROM
int romAddress = CALIB_FLASH_OFFSET;
bool foundCalib = false;


adafruit_bno055_offsets_t calibrationData;
sensor_t sensor;

/*
*  Look for the sensor's unique ID in ROM.
*  Isn't foolproof, but it's better than nothing.
*/
bno.getSensor(&sensor);
//i have no idea if this works; goes to the offset in ROM to find the saved sensor ID
if (*(int32_t *)(XIP_BASE + romAddress) != sensor.sensor_id)
{
  Serial.println("\nNo Calibration Data for this sensor exists in ROM");
  delay(500);
}
else
{
  Serial.println("\nFound Calibration for this sensor in ROM.");
 
  //increment working memory address to the calibration data
  romAddress += FLASH_PAGE_SIZE;
  calibrationData = *(adafruit_bno055_offsets_t *)(XIP_BASE + romAddress);

  bno.setSensorOffsets(calibrationData);

  Serial.println("\n\nCalibration data loaded into BNO055");
  foundCalib = true;
}

delay(250);

//Use external crystal for better accuracy, example code claims this must be done after loading calibration data
bno.setExtCrystalUse(true);

//complete calibration
sensors_event_t event;
bno.getEvent(&event);
if (!foundCalib)
{
    Serial.println("Please Calibrate Sensor: ");
    while (!bno.isFullyCalibrated())
    {
        bno.getEvent(&event);
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }
}

Serial.println("\nFully calibrated!");
uint8_t newCalib[22];
bno.getSensorOffsets(newCalib); //Get calibration offsets
Serial.println("\n\nStoring calibration data to ROM...");

//back to sensor id location
romAddress = CALIB_FLASH_OFFSET;
bno.getSensor(&sensor);
//turn sensor into an array of uint8_t
uint8_t bnoID[FLASH_PAGE_SIZE];
bnoID[3] = (uint8_t)sensor.sensor_id;
bnoID[2] = (uint8_t)(sensor.sensor_id>>=8);
bnoID[1] = (uint8_t)(sensor.sensor_id>>=8);
bnoID[0] = (uint8_t)(sensor.sensor_id>>=8);

//Disable interrupts while writing to ROM
uint32_t ints = save_and_disable_interrupts();
flash_range_erase (romAddress, FLASH_SECTOR_SIZE);
flash_range_program (romAddress, bnoID, FLASH_PAGE_SIZE);
//increment
romAddress += FLASH_PAGE_SIZE;
flash_range_program (romAddress, newCalib, FLASH_PAGE_SIZE);
restore_interrupts (ints);

Serial.println("Data stored to ROM.");

//End BNO055 Setup

//Repeatedly check accelerometer and barometer altitude here:

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
  
//delay(5000);

//analogWrite(LED_PWM,10);


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