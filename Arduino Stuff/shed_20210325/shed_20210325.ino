/***************************************************************************************************
  TSL2591 Digital Light Sensor (this is for shutting off the light inside the dog kennel)
***************************************************************************************************/

#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
/**********************************************************************************************/
/*Adafruit Data Shield
 *********************************************************************************************
   SD card attached to SPI bus as follows:
 ** UNO:  MOSI - pin 11, MISO - pin 12, CLK - pin 13, CS - pin 4 (CS pin can be changed)
  and pin #10 (SS) must be an output
 ** Mega:  MOSI - pin 51, MISO - pin 50, CLK - pin 52, CS - pin 4 (CS pin can be changed)
  and pin #52 (SS) must be an output
 ** Leonardo: Connect to hardware SPI via the ICSP header
     Pin 4 used here for consistency with other Arduino examples
 **On the Ethernet Shield, CS is pin 4. Note that even if it's not
   used as the CS pin, the hardware CS pin (10 on most Arduino boards,
   53 on the Mega) must be left as an output or the SD library
   functions will not work. */
#include <SPI.h>
#include <Wire.h> //This is the library that helps the Arduino with i2c
#include "SD.h" // This is the library for talking to the card
File dataFile;
const int chipSelect = 10; // this is the CS for the data logger (there can be no other pins connected to this)
unsigned long ir;
/*********************************************************************************************
   Time Stamp
 * ******************************************************************************************/
#include "RTClib.h" //this is the library the Arduino uses to chat with the real time clock
RTC_PCF8523 rtc;

/******************************************************************************************
   Temp, Humid, Baro BMP280 (outside sensor)
 *******************************************************************************************/
#include <Adafruit_BME280.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 9

#define SEALEVELPRESSURE_HPA (1013.25)

//Adafruit_BME280 bme(BME_CS); // hardware SPI
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

/*********************************************************************************************
 * Waterproof Temp Sensore DS18B20
 *********************************************************************************************/
#include <OneWire.h>
#include <DallasTemperature.h>

// all of the data is going to the one pin. Since the temo sensor is digital, all of the sensors have their own address
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

DeviceAddress shed_temp = { 0x28, 0xD2, 0xB9, 0x3E, 0x46, 0x20, 0x1, 0x2A };// this is the shed
DeviceAddress cricket_temp = { 0x28, 0x8A, 0x95, 0xF7, 0x4B, 0x20, 0x1, 0xC6 };// this is cricket
DeviceAddress outside_temp = { 0x28, 0x71, 0x8, 0x10, 0x4C, 0x20, 0x1, 0xA2 };// This is the outside sensor
DeviceAddress jango_temp = {0x28, 0xE7, 0xDF, 0xE2, 0x4B, 0x20, 0x1, 0xA2};// this is Jango




void setup()
{
  Serial.begin(115200);// This has to be 115200 for the DS18B20 sensors to work in series
  sensors.begin(); // this is for the DS18B20

  Serial.println(F("Starting Adafruit TSL2591 Test!"));

  if (tsl.begin())
  {
    Serial.println(F("Found a TSL2591 sensor"));
  }
  else
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }

  while (!Serial);   // time to get serial running
  Serial.println(F("BME280 test"));

  unsigned status;

  // default settings
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  Serial.println("-- Default Test --");


  Serial.println();


  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  //pinMode(SS, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("SDCard failed, or not present");
    // don't do anything more:
    while (1) ;
  }
  Serial.println("card initialized.");

  // Open up the file we're going to log to!
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      dataFile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }
  //dataFile = SD.open("datalog.csv", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error creating file");
    // Wait forever since we cant write data
    while (1) ;
  }
  Serial.print("Logging to: ");
  Serial.println(filename);
}
/***********************************************************************
   Time Stamp
 * ********************************************************************/
void timeStamp () {
  // while (!Serial) {
  //delay(1);
  //}

  //Serial.begin(9600);
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);

  dataFile.print(now.year(), DEC);
  dataFile.print('/');
  dataFile.print(now.month(), DEC);
  dataFile.print('/');
  dataFile.print(now.day(), DEC);
  dataFile.print(") ");
  dataFile.print(",");
  dataFile.print(now.hour(), DEC);
  dataFile.print(':');
  dataFile.print(now.minute(), DEC);
  dataFile.print(':');
  dataFile.print(now.second(), DEC);
}
/**************************************************************************
    Light Sensor TSL2591
     Configures the gain and integration time if required
**************************************************************************/
void advancedRead() {
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.print(tsl.calculateLux(full, ir), 6);

  dataFile.print(",");
  dataFile.print(ir);
  dataFile.print(",");
  dataFile.print(full);
  dataFile.print(",");
  dataFile.print(full - ir);
  dataFile.print(",");
  dataFile.println(tsl.calculateLux(full, ir), 6);
}

/**************************************************************************
    Temp Hum Baro Sensor BME280
**************************************************************************/
void TempHumBaroRead() {

  Serial.print(" Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.print(" *C");

  Serial.print(" Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.print(" hPa");

  Serial.print(" Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.print(" m");

  Serial.print(" Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
}

void Temps() {
    //Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.println("DONE");
  
  Serial.print("Shed Temperature(*C): ");
  Serial.print(sensors.getTempC(shed_temp)); 
  Serial.print("(*F): ");
  Serial.print(sensors.getTempF(shed_temp)); 
 
  Serial.print("Cricket Temperature(*C): ");
  Serial.print(sensors.getTempC(cricket_temp)); 
  Serial.print("(*F): ");
  Serial.print(sensors.getTempF(cricket_temp)); 
  
  Serial.print("Outside Temperature(*C): ");
  Serial.print(sensors.getTempC(outside_temp)); 
  Serial.print("(*F): ");
  Serial.print(sensors.getTempF(outside_temp)); 

  Serial.print("Jango Temperature (*C): ");
  Serial.print(sensors.getTempC(jango_temp)); 
  Serial.print("(*F): ");
  Serial.print(sensors.getTempF(jango_temp));
}
void loop()
{

  timeStamp();
  advancedRead();
  TempHumBaroRead();
  Temps();


  // The following line will 'save' the file to the SD card after every
  // line of data - this will use more power and slow down how much data
  // you can read but it's safer!
  // If you want to speed up the system, remove the call to flush() and it
  // will save the file only every 512 bytes - every time a sector on the
  // SD card is filled with data.
  dataFile.flush();

  delay(1500);
}
