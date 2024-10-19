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

/*
0 - Startup
1 - Awaiting launch
2 - Ascent
3 - Braking
4 - Landed
*/
int flightState = 0;

//time tracking
unsigned long loopStart = 0; //time (millis) set at beginning of loop, for loop time calculation
unsigned long loopTime; //delta-t (millis) calculated at end of loop
unsigned long timeLoopTimePrinted = 0; //time(millis) set every time the loop time is printed to serial

//used for sensor output during calibration
const double degToRad = 57.295779513;

//SD Card reader stuff:
#define CS_PIN 17  //Connect Chip Select pin to GPIO 17 on the Pi Pico
//end of sd card stuff

//offset from start of flash memory for BNO055 calibration data
//we can access this at XIP_BASE + 1024k.

//Aerobrake and LED pin assignments
#define NUM_LEDS 4

#define LED_PWM 22

/*Here is what we will use each LED for:
  D1 [LED at index 0] --> Status/info of Pi Pico, or other 
  D2 [LED at index 1] --> Status/info of BMP388
  D3 [LED at index 2] --> Status/info of BNO055
  D4 [LED at index 3] --> Status/info of Micro SD Card adapter
    Yellow: calibration file not found
*/

CRGB leds[NUM_LEDS];

Servo BRAKE_PWM;
//end of pwm stuff

//Sensor assignments and sensor stuff
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BMP3XX bmp;
#define SEALEVELPRESSURE_HPA (1013.25) //Sea level pressure for calculating altitude
float AltOffset = 0; //Zero altitude

//define BNO055 info functions
void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);


//end of sensor stuff

int pos = 0;

/**************************************************************************

SETUP LOOP

**************************************************************************/
void setup() {

  //initialize motor
  BRAKE_PWM.writeMicroseconds(500);
  BRAKE_PWM.attach(9);
  //pinMode(BRAKE_PWM, OUTPUT);

  //Connect to serial monitor
  Serial.begin(115200);
  //while (!Serial) delay(10); //Wait to start communiticating until serial opens
  delay(250);
  Serial.println("Connected to Flight Computer");
  Serial.println("");
  
  //Add led element
  FastLED.addLeds<NEOPIXEL, LED_PWM>(leds, NUM_LEDS);  //Defaults to GRB color order
  FastLED.setBrightness(100);
  //LED test
  leds[0] = CRGB(255,0,0);
  leds[1] = CRGB(255,0,0);
  leds[2] = CRGB(255,0,0);
  leds[3] = CRGB(255,0,0);
  FastLED.show(); 
  delay(300);
  leds[0] = CRGB(0,255,0);
  leds[1] = CRGB(0,255,0);
  leds[2] = CRGB(0,255,0);
  leds[3] = CRGB(0,255,0);
  FastLED.show(); 
  delay(300);
  leds[0] = CRGB(0,0,255);
  leds[1] = CRGB(0,0,255);
  leds[2] = CRGB(0,0,255);
  leds[3] = CRGB(0,0,255);
  FastLED.show(); 
  delay(1000);
  for (int i = 0; i <= 3; i++) {
    leds[i] = CRGB(0,0,0);
  }
  FastLED.show();
  //Initialize communication busses
  SPI.begin();
  Wire.begin();

  if (!bno.begin()) { //Display error message if not able to connect to IMU
      Serial.println("Error: No IMU found on I2C bus");
      leds[2] = CRGB(255,0,0);
      while (1);
    }

  if (!bmp.begin_I2C()) {  //Display error message if not able to connect to Barometer, defautls to I2C mode (what we are using)
      Serial.println("Error: No Barometer found on I2C bus");
      leds[1] = CRGB(255,0,0);
      while (1);
    }
  if (!SD.begin(CS_PIN)) { //Display error message if not able to connect to Micro SD card adapter
      Serial.println("Error: Unable to initialize SD card, no adapter found on SPI bus");
      leds[3] = CRGB(255,0,0);
      FastLED.show();
      while (1); 
    }

  //Pressure sensor settings
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  //Zero sensor at altitude
  AltOffset = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("Zeroed at altitude ");
  Serial.print(AltOffset);
  Serial.println(" m");
  
  delay(100);

  //Begin BNO055 setup

  //Search for BNO055 calibration data
  bool foundCalib = false;

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
  File calibInfo;
  /*
  *  Check if sensor calibration present.
  */
  bno.getSensor(&sensor);
  if (!SD.exists("calibration.bin"))
  {
    Serial.println("No calibration data on SD card");
    leds[2] = CRGB(255,255,0);
    FastLED.show();  
  }
  else
  {
    Serial.println("\nFound Calibration on SD card.");
    calibInfo = SD.open("calibration.bin", FILE_READ);

    //read file's bytes and shove into offsets
    calibInfo.read((uint8_t *)&calibrationData, sizeof(calibrationData));
    displaySensorOffsets(calibrationData);
    calibInfo.close();

    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
    SD.remove("calibration.bin");
  }

  delay(250);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  //Use external crystal for better accuracy, example code claims this must be done after loading calibration data
  bno.setExtCrystalUse(true);

  //bno.setMode(OPERATION_MODE_IMUPLUS); // set BNO to not use magnetometer

  //complete calibration
  sensors_event_t event;
  bno.getEvent(&event);
  if (!foundCalib)
  {
    Serial.println("Please Calibrate Sensor: ");

    //bno.setMode(OPERATION_MODE_NDOF);
    while (!bno.isFullyCalibrated())
    {
      bno.getEvent(&event);

      imu::Vector<3> euler = bno.getQuat().toEuler();
      
      double x = euler.y() * degToRad;
      double y = euler.z() * degToRad;
      double z = euler.x() * degToRad;
      
      Serial.print("X: ");
      Serial.print(x, 4);
      Serial.print(" Y: ");
      Serial.print(y, 4);
      Serial.print(" Z: ");
      Serial.print(z, 4);
      Serial.print("\t\t");

      /* Optional: Display calibration status */
      displayCalStatus();

      /* New line for the next sample */
      Serial.println("");

      /* Wait the specified delay before requesting new data */
      delay(500);
    }
  }

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");

  //Get calibration offsets as the offsets data type to display
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  //write to file
  calibInfo = SD.open("calibration.bin", FILE_WRITE);
  calibInfo.write((uint8_t *)&newCalib, sizeof(newCalib));
  calibInfo.close();
  Serial.println("Saved calibration data to SD card");
  delay(100);

  bno.setMode(OPERATION_MODE_IMUPLUS); // set BNO to not use magnetometer

  delay(50);

  leds[0] = CRGB(0,255,0);
  leds[1] = CRGB(0,255,0);
  leds[2] = CRGB(0,255,0);
  leds[3] = CRGB(0,255,0);
  FastLED.show(); 
  delay(1000);

  //End BNO055 Setup

  //Repeatedly check accelerometer and barometer altitude:
  flightState = 1;
  sensors_event_t accelerometerData;
  do {
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    
    //print observed acceleration
    Serial.print("Upwards acceleration: ");
    Serial.print(-accelerometerData.acceleration.y);
    Serial.println(" m/s^2");
    delay(BNO055_SAMPLERATE_DELAY_MS);

  } while ((accelerometerData.acceleration.y >= -20) && ((bmp.readAltitude(SEALEVELPRESSURE_HPA) - AltOffset) <= 10));

  flightState = 2;
  Serial.println("Launch Detected!");
  fill_solid(leds, NUM_LEDS, CRGB(160, 43, 147));  // Fill all LEDs with Purple
  FastLED.show();
}

/**************************************************************************

MAIN LOOP

**************************************************************************/
void loop() {
  loopTime = millis() - loopStart;
  loopStart = millis();

  //delay(500);
  
  //Print loop time every half a second
  if ((millis() - timeLoopTimePrinted) >= (500 * 1000)) {
    timeLoopTimePrinted = millis();
    Serial.print("Loop time: ");
    Serial.print(loopTime);
    Serial.println(" milliseconds");
  } 
}


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(100);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(100);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
    delay(100);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.println(calibData.mag_radius);
    delay(100);
}