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
int loopsElapsed = 0; //# of loops executed since loopTime measured

unsigned long loopStartTime = 0; //time (millis) set at beginning of loop, for loop time calculation
unsigned long loopTime; //delta-t (millis) calculated at end of loop
unsigned long loopPrintTimer = 0; //time (millis) set every time the loop time is printed to serial

unsigned long launchTime; //time (millis) set once launch detected
unsigned long logTime; //time (millis) set on each sensor read
unsigned long altIndexTimer; //time (milllis) set every time a new altitude is indexed for land detect

unsigned long fileSaveTimer; //time (millis) since last file save

//used for sensor output during calibration
const double degToRad = 57.295779513;

//SD Card reader stuff:
#define CS_PIN 17  //Connect Chip Select pin to GPIO 17 on the Pi Pico
bool calibOnSD = true;
char *csvName;
char *getCSVName(unsigned int i);
/* CSV file format: comma delineated
each term in a row is separated by a comma and each row is separated by a newline character (\n)

Note: need to swap some values around as the bno055 coordinate axes are different
Following right hand rule, for orientation from quaternion: rocket x is sensor y, rocket y is sensor -z, rocket z is sensor x
For vectors: rocket x is sensor z, rocket y is sensor -x, rocket z is sensor -y
rocket x is pointing perpendicular to the board, front face, rocket z is straight up

Data: time, alt, x accel, y accel, z accel, x angle, y angle, z angle, x angvel, y angvel, z angvel, pressure, temp
*/
File dataFile;
const char headerString[] = "time (ms),altitude (m),xaccel (m/s^2),yaccel (m/s^2),zaccel (m/s^2),xangle (deg),yangle (deg),zangle (deg),xangvel (dps),yangvel (dps),zangvel (dps),pressure (Pa),temperature (deg C)";
void comma() {
  dataFile.print(',');
  }
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
    Green:  OK
    Yellow: Awaiting calibration
    Purple: Ascent
    Orange: Braking
    Blue:   Landed
    Red:    Fucked
*/
CRGB leds[NUM_LEDS];

Servo BRAKE_PWM;
//end of pwm stuff

//Sensor assignments and sensor stuff
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#define SENSOR_SAMPLE_MIN_DELAY (10)

Adafruit_BMP3XX bmp;
#define SEALEVELPRESSURE_HPA (1013.25) //Sea level pressure for calculating altitude
float altOffset; //Zero altitude
#define AEROBRAKE_DEPLOY (800) //Altitude at which to deploy aerobrakes

float altIndex[3]; //New altitude indexed in every 5 seconds after launch

//define BNO055 info functions
void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);

bool allButMagCalibrated();

bool badSensorComms = false;
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
  delay(2000);
  Serial.println("Connected to Flight Computer");
  Serial.println("");
  //Add led element
  FastLED.addLeds<NEOPIXEL, LED_PWM>(leds, NUM_LEDS);  //Defaults to GRB color order
  FastLED.setBrightness(25);
  //LED test
  fill_solid(leds, NUM_LEDS, CRGB(255,0,0));
  FastLED.show(); 
  delay(300);
  fill_solid(leds, NUM_LEDS, CRGB(0,255,0));
  FastLED.show(); 
  delay(300);
  fill_solid(leds, NUM_LEDS, CRGB(0,0,255));
  FastLED.show(); 
  delay(300);
  FastLED.clear();
  FastLED.show();
  //Initialize communication busses
  SPI.begin();
  Wire.begin();
  do {
    if (!bmp.begin_I2C()) {  //Display error message if not able to connect to Barometer, defautls to I2C mode (what we are using)
      Serial.println("Error: No Barometer found on I2C bus");
      leds[1] = CRGB(255,0,0);
      badSensorComms = true;
    }
    else leds[1] = CRGB(0,255,0);
    FastLED.show();

    if (!bno.begin()) { //Display error message if not able to connect to IMU
      Serial.println("Error: No IMU found on I2C bus");
      leds[2] = CRGB(255,0,0);
      badSensorComms = true;
    }
    else leds[2] = CRGB(255,240,0);
    FastLED.show();

    if (!SD.begin(CS_PIN)) { //Display error message if not able to connect to Micro SD card adapter
      Serial.println("Error: Unable to initialize SD card, no adapter found on SPI bus");
      leds[3] = CRGB(255,0,0);
      badSensorComms = true;
    }
    else leds[3] = CRGB(0,255,0);
    FastLED.show();
    
    if (SD.begin(CS_PIN) && bmp.begin_I2C() && bno.begin()) {
      badSensorComms = false;
      leds[1] = CRGB(0,255,0);
      leds[2] = CRGB(255,240,0);
      leds[3] = CRGB(0,255,0);
    }
    FastLED.show();
  } while (badSensorComms); //Check through all sensors once, if there's a problem keep checking
  //Pressure sensor settings
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  //Zero sensor at altitude
  float tempPressure = bmp.readPressure();
  delay(500); //Let sensor initialize
  altOffset = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("\nZeroed at altitude ");
  Serial.print(altOffset);
  Serial.println(" m");
  
  //Initialize CSV on SD card
  csvName = getCSVName(0);
  if (csvName == 0) {
    leds[3] = CRGB(255,0,0);
    Serial.println("\nMemory allocation error while trying to find a CSV file name");
    Serial.println("How many fucking times did you run this thing");
    while(1);
  }
  Serial.print("\nInitializing CSV file ");
  Serial.println(csvName);
  dataFile = SD.open(csvName, FILE_WRITE);
  if (dataFile) {
    dataFile.println(headerString);
    dataFile.close();
  }
  else {
    leds[3] = CRGB(255,0,0);
    Serial.println("\nError opening data file, could not write headers");
    delay(500);
  }
 

  //Begin BNO055 setup

  //BNO055 calibration

  adafruit_bno055_offsets_t calibrationData;
  File calibInfo;
  bool foundCalib = false;
  /*
  *  Check if sensor calibration present.
  */
  if (!SD.exists("calibration.bin"))
  {
    Serial.println("\nNo calibration data on SD card");
    calibOnSD = false;
  }
  else
  {
    Serial.println("\nFound Calibration on SD card.");
    calibInfo = SD.open("calibration.bin", FILE_READ);
    if (calibInfo) {
      //read file's bytes and shove into offsets
      int calibBytesRead = calibInfo.read((uint8_t *)&calibrationData, sizeof(calibrationData)); //Returns # of bytes read, or -1 if error
      if (calibBytesRead == sizeof(calibrationData)) {
        displaySensorOffsets(calibrationData);
        calibInfo.close();

        Serial.println("Restoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);
        Serial.println("Calibration data loaded into BNO055");
        foundCalib = true;

        //Display new offsets for verification
        adafruit_bno055_offsets_t newData;
        Serial.println("Offsets currently on sensor:");
        if (bno.getSensorOffsets(newData)) {
          displaySensorOffsets(newData);
        }
        else {
          Serial.println("\nFailed to read new offsets");
        }
      }
      else if (calibBytesRead == -1){ //Error while reading calibration data
        Serial.println("\nCalibration data read failed");
        leds[3] = CRGB(255,0,0);
        FastLED.show();
      }
      else { //Mismatch of data bytes read
        Serial.print("\nOnly read ");
        Serial.print(calibBytesRead);
        Serial.print(" bytes out of ");
        Serial.println(sizeof(calibrationData));
        leds[3] = CRGB(255,0,0);
        FastLED.show();
      }
    }
    else { //File open error
      Serial.println("\nCalibration data open failed");
      leds[3] = CRGB(255,0,0);
      FastLED.show();
    }
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  //Use external crystal for better accuracy, example code claims this must be done after loading calibration data
  bno.setExtCrystalUse(true);

  bno.setMode(OPERATION_MODE_IMUPLUS); // set BNO to not use magnetometer

  //complete calibration
  if (!foundCalib) {
    leds[2] = CRGB(255,200,0);
    FastLED.show();
    Serial.println("\nPlease Calibrate Sensor: ");

    //bno.setMode(OPERATION_MODE_NDOF);

    while (!allButMagCalibrated())
    {
      //Get quaternions and convert to euler angles for better accuracy
      imu::Vector<3> euler = bno.getQuat().toEuler();
      
      double x = euler.y() * degToRad;
      double y = -euler.z() * degToRad;
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
  //Get calibration offsets to write and display
  if(!calibOnSD) {
    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");

    adafruit_bno055_offsets_t newCalib;
    if (bno.getSensorOffsets(newCalib)) {
      displaySensorOffsets(newCalib);
      //write to file
      calibInfo = SD.open("calibration.bin", FILE_WRITE);
      if (calibInfo) {
        calibInfo.write((uint8_t *)&newCalib, sizeof(newCalib));
        calibInfo.close();
        Serial.println("Saved calibration data to SD card");
        delay(100);
      }
      else {
        Serial.println("\nSD card write failure, offsets may be borked!");
        leds[3] = CRGB(255,0,0);
        FastLED.show();
        delay(2000);
      }
    }
    else {
      Serial.println("\nOffset retrieval failure; cannot save offsets");
      leds[3] = CRGB(255,0,0);
      FastLED.show();
      delay(2000);
    }
  }
  //bno.setMode(OPERATION_MODE_IMUPLUS); // set BNO to not use magnetometer

  fill_solid(leds, NUM_LEDS, CRGB(0, 255, 0));
  FastLED.show(); 

  //End BNO055 Setup

  //Repeatedly check accelerometer and barometer altitude:
  flightState = 1;
  sensors_event_t accelerometerData;
  Serial.println("");
  do {
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    
    //print observed acceleration
    Serial.print("Upwards acceleration: ");
    Serial.print(-accelerometerData.acceleration.y);
    Serial.println(" m/s^2");
    delay(SENSOR_SAMPLE_MIN_DELAY);

  } while ((accelerometerData.acceleration.y >= -20) && ((bmp.readAltitude(SEALEVELPRESSURE_HPA) - altOffset) <= 10));
  
  //Launch detect
  launchTime = millis();
  flightState = 2;
  Serial.println("\n*****************");
  Serial.println("Launch Detected!");
  Serial.println("*****************");
  fill_solid(leds, NUM_LEDS, CRGB(160, 43, 147));  // Fill all LEDs with Purple
  FastLED.show();
}

/**************************************************************************

MAIN LOOP

**************************************************************************/
void loop() {
  loopStartTime = millis();
  
  //Flight conditions

  //Aerobrake Deploy
  if ((flightState == 2) && ((bmp.readAltitude(SEALEVELPRESSURE_HPA) - altOffset)>= AEROBRAKE_DEPLOY)) {
    flightState = 3;
    Serial.println("\nAt altitude, aerobrakes deployed");
    BRAKE_PWM.writeMicroseconds(2500);
    fill_solid(leds, NUM_LEDS, CRGB(255, 100, 0));
    FastLED.show();
  }

  //Landing detection
  //Index new altitude every five seconds
  if ((millis() - altIndexTimer) >= 5000) {
    altIndexTimer = millis();
    //somewhat excessive for shifting two values but it's scaleable so whatever
    for (int i = (sizeof(altIndex) / sizeof(float)) - 1; i > 0; i--) {
      altIndex[i] = altIndex[i - 1];
    }
    altIndex[0] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }

  if ( //If highest alt in array and lowest alt in array closer than 10 meters, landed 
    (flightState != 4) && ((millis() - launchTime) >= 20000) && //Can't have just launched or it would trigger immediately
    ((max(altIndex[0],max(altIndex[1], altIndex[2])) -
    min(altIndex[0],min(altIndex[1], altIndex[2]))) <= 10))
    {
    flightState = 4;
    Serial.print("\nDetected landing at ");
    Serial.print(altIndex[0] - altOffset);
    Serial.println(" meters");
    fill_solid(leds, NUM_LEDS, CRGB(0, 0, 255));
    FastLED.show();
  }

  //Perform logging every sample period
  if ((flightState != 4)) {
    while ((millis() - logTime) < SENSOR_SAMPLE_MIN_DELAY) delay(1); //Make sure data logged no more than 100 times/sec
    
    //Pull data

    unsigned long logTime = millis();

    //BNO
    sensors_event_t acceleration, angVel;
    imu::Vector<3> orientation = bno.getQuat().toEuler();
    bno.getEvent(&acceleration, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&angVel, Adafruit_BNO055::VECTOR_GYROSCOPE);

    //BMP
    float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - altOffset;
    float pressure = bmp.readPressure();
    float temperature = bmp.readTemperature();

    //Record data
    dataFile = SD.open(csvName, FILE_WRITE);
    if (dataFile) {
      dataFile.print(logTime - launchTime);
      comma();
      dataFile.print(altitude);
      comma();
      dataFile.print(acceleration.acceleration.z);
      comma();
      dataFile.print(-acceleration.acceleration.x);
      comma();
      dataFile.print(-acceleration.acceleration.y);
      comma();
      dataFile.print(orientation.y() * degToRad);
      comma();
      dataFile.print(-orientation.z() * degToRad);
      comma();
      dataFile.print(orientation.x() * degToRad);
      comma();
      dataFile.print(angVel.gyro.z * degToRad);
      comma();
      dataFile.print(-angVel.gyro.x * degToRad);
      comma();
      dataFile.print(-angVel.gyro.y * degToRad);
      comma();
      dataFile.print(pressure);
      comma();
      dataFile.print(temperature);
      dataFile.print('\n');
      dataFile.close();
    }
    else {
      leds[3] = CRGB(255, 0, 0);
      Serial.println("\nFailed to open CSV file");
    }
  }
  //Print average loop time every two seconds
  if ((millis() - loopPrintTimer) >= (2000)) {
    loopTime = (millis() - loopPrintTimer)/loopsElapsed;
    loopPrintTimer = millis();
    loopsElapsed = 0;
    Serial.print("\nLoop time: ");
    Serial.print(loopTime);
    Serial.println(" milliseconds");
  }
  else loopsElapsed ++;
}

/*Try to get a name for a CSV file like data0.csv, if already present try the next number.
e.g. i=0 and data0.csv data7.csv are already present, returns data8.csv

Returns 0 if allocation fails*/
char * getCSVName(unsigned int i) {
  //convert int to ASCII
  int mag1 = i;
  int mag2 = i;
  int len = 0;
  
  while (mag1 > 0) {
    len++;
    mag1 /= 10;
  }

  if (len == 0) len++;

  char istr[len + 1];
  for (int j = len - 1; j >= 0; j--) {
    istr[j] = mag2 % 10 + '0';
    mag2 /= 10;
  }
  istr[len] = '\0';
  char datastr[] = "data";
  char csvstr[] = ".csv";
  
  //Allocate memory for filename
  size_t strlength = 9 + len; //Length of string is 4 (data) + decimal length of number + 5 (.csv\0)
  char *filename = (char *) malloc(strlength);
  if (filename == NULL) return 0; //null pointer means something is fucked

  //Glue string together and test
  strcpy(filename, datastr); //Don't use strcat here becuase it's not initialized yet
  strcat(filename, istr);
  strcat(filename, csvstr);
  if (SD.exists(filename)) {
    //filename would overwrite existing file, free the memory and try the next one
    free(filename);
    //Try next one
    return getCSVName(i + 1);
  }
  else return filename;
}

//Check relevant calibration
bool allButMagCalibrated() {
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  return (system == 3) && (gyro == 3) && (accel == 3);
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
