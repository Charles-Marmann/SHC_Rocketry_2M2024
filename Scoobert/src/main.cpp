#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/**************************************************************************/
/*
  Start BNO pre-setup
*/
/**************************************************************************/


#define BNO055_SAMPLERATE_DELAY_MS (100)


// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
File testFile;
File logFile;

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
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
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
  delay(500);
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
}




/**************************************************************************/
/*
  End BNO pre-setup
*/
/**************************************************************************/




//#define BMP_SCK 13
//#define BMP_MISO 12
//#define BMP_MOSI 11
//#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

void setup() {

/**************************************************************************/
/*
 Begin BNO setup
*/
/**************************************************************************/

    while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);

/**************************************************************************/
/*
 End BNO setup
*/
/**************************************************************************/

/**************************************************************************/
/*
  Start BMP setup
*/
/**************************************************************************/

Serial.begin(9600);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();


  if (!SD.begin(17))
  {
    Serial.println("Failed");
    while(1);
}

  testFile = SD.open("test.txt", FILE_WRITE);

  if (testFile)
  {
    Serial.print("Writing Test");
    testFile.println("Big Titty Goth Bitches and Dommy Mommies");
    testFile.close();
    Serial.println("Done.");
 } else
  {
    Serial.println("Error opening the test file");
  }
}


/**************************************************************************/
/*
  end BMP setup
*/
/**************************************************************************/

//void loop() {
  //logFile = SD.open("logfile.txt", FILE_WRITE);
 // if (logFile)
 // {
  /* Get a new sensor event */ 
  //sensors_event_t event; 
 // bno.getEvent(&event);
  
 // logFile << event.orientation.x << " " << event.orientation.y << " " << event.orientation.z;
 // OUTPUT = std::to_string(event.orientation.x) + " " + std::to_string(event.orientation.y) + " " + std::to_string(event.orientation.z);
 // logFile << std::endl;
  
  //delay(100);
  
  //if (! bmp.performReading()) {
  //  Serial.println("Failed to perform reading :(");
  //  return;
  //}
 // logFile.print("Temperature = ");
 // logFile.print(bmp.temperature);
  //logFile.println(" *C");

  //logFile.print("Pressure = ");
  //logFile.print(bmp.pressure / 100.0);
 // logFile.println(" hPa");

 // logFile.print("Approx. Altitude = ");
 // logFile.print(bmp.readAltitude(1013.25));
 // logFile.println(" m");

  //logFile.println();
 // logFile.close();
 // delay(2000);
 // }
//}

void loop() {
  logFile = SD.open("logfile.csv", FILE_WRITE);  // Change filename to .csv
  if (logFile) {
    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);

    // Log orientation data in CSV format
    logFile.print(event.orientation.x);
    logFile.print(",");  // Add comma as separator
    logFile.print(event.orientation.y);
    logFile.print(",");  // Add comma as separator
    logFile.print(event.orientation.z);
    logFile.print(",");  // Add comma for next field (temperature)
    
    // Read BMP data
    if (!bmp.performReading()) {
      Serial.println("Failed to perform reading :(");
      logFile.close(); // Ensure to close logFile if reading fails
      return;
    }

    // Log temperature, pressure, and altitude in CSV format
    logFile.print(bmp.temperature);
    logFile.print(",");  // Add comma as separator
    logFile.print(bmp.pressure / 100.0);
    logFile.print(",");  // Add comma as separator
    logFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    logFile.println();  // End the line

    logFile.close(); // Always close the file when done
  } else {
    Serial.println("Error opening the logfile");
  }

  delay(100); //logs the data every 100 milliseconds, subject to change
}