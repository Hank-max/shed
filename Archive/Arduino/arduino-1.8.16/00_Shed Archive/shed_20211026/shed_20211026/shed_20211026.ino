


const int dk_lights = 33; // this is the pin for the 12V relay that is contorlling the lights in the dog kennel.
const int dk_fans = 25; //this is the pin on the board where the relay is attached for the fans in the dog kennel
const int shed_fan = 26;// this is to the relay for the shed exhaust fan
const int comp_fans = 24;// this is the fans in the computer box
const int chipSelect = 10; // this is the CS for the data logger, which is currently not being used.
const int ONE_WIRE_BUS = 2; // this is the one wire (pin) that is used to read the all of the DS18B20 sensors (shed, outside, cricket, jango)
const int BME_SCK = 13;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_MISO = 12;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_MOSI = 11;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_CS = 9; // this is the chip select for the BMP280 (temp, hum, baro) sensor
const unsigned long SECONDS = 1000;
const unsigned long MIN = (60 * SECONDS);

/***************************************************************************************************
  TSL2591 Digital Light Sensor (this is for shutting off the light inside the dog kennel)
***************************************************************************************************/

#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
unsigned long ir;
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

//#define LOG_INTEVAL 1000 //This is how many milli seconds between the sensor readings (i woul like to figure out these features, but currently not using.
//#define ECHO_TO_SERIAL 1 //this is if i want to echo to the serial monitor (1 is on and 0 is off)
//#define redLEDpin 3 // this is for the error below. I will need to go back and change the pin.
void error(char *str)// the error function is uded if there is something bad happening (i.e. no micro SD card is installed. It will sit in the "while(1) loop forever until the issue is corrected
{
  Serial.print("error: ");
  Serial.println(str);

  // red LED indicates error
  //digitalWrite(redLEDpin, HIGH);

  while (1);
}

/*********************************************************************************************
   Time Stamp
 * ******************************************************************************************/
#include "RTClib.h" //this is the library the Arduino uses to chat with the real time clock
RTC_PCF8523 rtc; // I found a different variable (RTC_DS1307) If the time is incorrect than i want to try this

/******************************************************************************************
   Temp, Humid, Baro BMP280 (outside sensor)
 *******************************************************************************************/
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

//Adafruit_BME280 bme(BME_CS); // hardware SPI
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

/*********************************************************************************************
   Waterproof Temp Sensore DS18B20
 *********************************************************************************************/
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with a OneWire device
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor

DeviceAddress shed_temp = { 0x28, 0xD6, 0xE4, 0x10, 0x4C, 0x20, 0x01, 0x80 };// this is the shed
DeviceAddress cricket_temp = { 0x28, 0x8A, 0x95, 0xF7, 0x4B, 0x20, 0x1, 0xC6 };// this is cricket
DeviceAddress outside_temp = {0x28, 0xD6, 0xE4, 0x10, 0x4C, 0x20, 0x01, 0x80 };// This is the outside sensor
DeviceAddress jango_temp = {0x28, 0xE7, 0xDF, 0xE2, 0x4B, 0x20, 0x1, 0xA2};// this is Jango

/************************************************************************************************
   DHT11 temp and humidity sensor
 ***********************************************************************************************/
#include "DHT.h"
#define sensor1 3
#define sensor2 4
#define DHTTYPE DHT11 //leaving this here because i may get the DHT22
DHT dht1(sensor1, DHTTYPE);
DHT dht2(sensor2, DHTTYPE);
/*********************************************************************************************************************
*********************************************************************************************************************
                                SETUP
*********************************************************************************************************************
*********************************************************************************************************************/
void setup()
{
  Serial.begin(9600);
  //Serial.begin(115200);// This has to be 115200 for the DS18B20 sensors to work in series
//delay(1);

  /**************************************************************************************************
     Data Logger Shield Setup
   *************************************************************************************************/
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("SDCard failed, or not present");
    // don't do anything more:
    return ;
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

  Wire.begin(); // i added this, so lets see if it works.
//  if (! rtc.begin()) {
//    Serial.println("Couldn't find RTC");
//    while (1);
//  }

  dataFile.print("date/time"); // this is going to be the top column of the excel file, so adjust accordingly
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
  tsl.begin();
//  if (tsl.begin())
//  {
//    Serial.println(F("Found a TSL2591 sensor"));
//  }
//  else
//  {
//    Serial.println(F("No sensor found ... check your wiring?"));
//    while (1);
//  }
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  pinMode(dk_lights, OUTPUT);// This will eventually be to the relay that controls the light
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print(F("Light Sensor (TSL2591) "));
  //Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] ")); //this is the time in millis when the measurement was taken, but i dont need because i have the time.
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);

  dataFile.print(",");
  dataFile.print("Light Sensor: ir full full-ir lux");
  dataFile.print(",");
  dataFile.print(ir);
  dataFile.print(",");
  dataFile.print(full);
  dataFile.print(",");
  dataFile.print(full - ir);//need to figure out what this is for.
  dataFile.print(",");
  dataFile.println(tsl.calculateLux(full, ir), 6);

  if (full < 25000) {
    digitalWrite(dk_lights, HIGH);//Changed this to HIGH because i wanted the lights off over the night
  }
  else {
    digitalWrite(dk_lights, HIGH);
  }

}

/**************************************************************************
 *************************************************************************
    All of the Temperature Sensors
**************************************************************************
**************************************************************************/
void Temps() {
  unsigned status;
  status = bme.begin();// default settings
  dht1.begin();
  dht2.begin();
  sensors.begin(); // this is for the DS18B20
  //delay(1000);// there may need to be a slight delay here for the sensor
  /********************************************
    BME280 temp sensor (Weather Station)
   ********************************************/

  //while (!Serial);   // time to get serial running
  // Serial.println(F("BME280 test"));


  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");

    while (1) delay(10);
  }

  Serial.print(" Temperature (BME280) = ");
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

  /**************************************************************************************************
     DS18B20 Temperature Sensors (OneWire) (Most Common Sensor)
   *************************************************************************************************/
  sensors.requestTemperatures(); // Send the command to get temperatures

  /**************************************************************************************************
    DHT11 Temperature Sensors (inside of the shed and computer case)
  *************************************************************************************************/
  float hum1 = dht1.readHumidity();
  float hum2 = dht2.readHumidity();
  float tempF1 = dht1.readTemperature(true);
  float tempF2 = dht2.readTemperature(true);

  /**************************************************************************************************
    Using the temperature data (may want to use matrix to get rid of default sensors)
  *************************************************************************************************/
  float cricket_var = sensors.getTempF(cricket_temp);
  float jango_var = sensors.getTempF(jango_temp);
  float outside_var = sensors.getTempC(outside_temp);
  float shed_var = sensors.getTempF(shed_temp);
  float ws_temp_var = bme.readTemperature();
  float comp_shed_var = dht2.readTemperature(true);
  
  pinMode(dk_fans, OUTPUT);
  pinMode(comp_fans, OUTPUT);
float dk_avg = (cricket_var + jango_var)/2;
float outside_avg = (outside_var + ws_temp_var)/2;
 float shed_avg = (comp_shed_var + shed_var)/2;
  
  //float dk_avg = ((sensors.getTempF(jango_temp)) + (sensors.getTempF(cricket_temp))) / 2; //this is the average temoature of the two sensors in the dog kennel in F
//  float outside_avg = ((bme.readTemperature()) + (sensors.getTempC(outside_temp))) / 2 ; // this is the average outside temp in Celsus
 // float shed_avg = ((dht2.readTemperature(true)) + (sensors.getTempF(shed_temp))) / 2;
  Serial.print("Dog Kennel AVG:");
  Serial.print(dk_avg);
  Serial.print("  Outside AVG:");
  Serial.print(outside_avg);
  Serial.print("  Shed AVG:");
  Serial.print(shed_avg);


  


  
//  Serial.print("Shed Temperature: ");
//  //Serial.print(sensors.getTempC(shed_temp));
//  //Serial.print("(*C): ");
//  Serial.print(sensors.getTempF(shed_temp));
//  Serial.print("(*F):  ");
//  Serial.print(tempF2);
//  Serial.print("   Computer Temp:  ");
//  Serial.print(tempF1);
//  Serial.print("  Cricket Temperature: ");
//  //Serial.print(sensors.getTempC(cricket_temp));
//  //Serial.print("(*C): ");
//  Serial.print(sensors.getTempF(cricket_temp));
//  Serial.print("(*F): ");
//  Serial.print("  Outside Temperature: ");
//  Serial.print(sensors.getTempC(outside_temp));
//  Serial.print("(*C): ");
//  Serial.print(sensors.getTempF(outside_temp));
//  Serial.print("(*F): ");
//  Serial.print("  Jango Temperature: ");
//  //Serial.print(sensors.getTempC(jango_temp));
//  //Serial.print("(*C): ");
//  Serial.print(sensors.getTempF(jango_temp));
//  Serial.println("(*F): ");

  //this is turning the dog kennel fan on and off based on the temperature outside and in the kennel (outside temp is in C)
  if (outside_avg >= 21) {
    if (dk_avg > 60) {
      digitalWrite(dk_fans, HIGH);
      Serial.println("  FAN IS ON");
    }
    else if (dk_avg <= 60) {
      digitalWrite(dk_fans, LOW);
      Serial.println("  FAN IS OFF");
    }
  }
  else if (outside_avg < 21) {
    if (dk_avg >= 65) {
      digitalWrite(dk_fans, HIGH);
      Serial.println("  FAN IS ON");
    }
    else if (dk_avg < 65) {
      digitalWrite(dk_fans, LOW);
      Serial.println("  FAN IS OFF");
    }
  }

  if (tempF1 <= 80) {
    digitalWrite (comp_fans, HIGH);
  }
  else {
    digitalWrite (comp_fans, LOW);
  }
pinMode(shed_fan, OUTPUT);
    if (shed_avg <= 85) {
      digitalWrite (shed_fan, HIGH);
  //    if (isnan(tempF1)) {
  //      digitalWrite(comp_fan, HIGH);
  //    }
  //    else {
  //
  //    }
    }
  //  else if {
  //  //digitalWrite (shed_fan, LOW);
}


void loop() {

  timeStamp();
  advancedRead();
  //TempHumBaroRead();
  Temps();



  // The following line will 'save' the file to the SD card after every
  // line of data - this will use more power and slow down how much data
  // you can read but it's safer!
  // If you want to speed up the system, remove the call to flush() and it
  // will save the file only every 512 bytes - every time a sector on the
  // SD card is filled with data.
  dataFile.flush();

  //delay(5 * MIN);
  delay(5000);
}
