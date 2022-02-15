

/**********************************************************************************************
   INT Relays
***********************************************************************************************/
const int dk_lights = 33; // 12V relay to lights in dog kennel.
const int dk_fans = 43; //SS relay for fans in dog kennel.
const int comp_fans = 34;//12V relay for fans in computer box.
const int shed_fan = 47; // SS relay for shed fan.
const int heater = 39; //SS relay for IR  Heater.

/**********************************************************************************************
   INT Loop Times
***********************************************************************************************/
const unsigned long SECONDS = 1000;
const unsigned long MIN = (60 * SECONDS);

unsigned long fan_previousMillis = 0;
unsigned long comp_previousMillis = 0;
unsigned long win_previousMillis = 0;
unsigned long heater_previousMillis = 0;
unsigned long print_previousMillis = 0;
unsigned long light_previousMillis = 0;

long win_time = 30 * SECONDS; //MIN;
long compfan_time = 30 * SECONDS;
long heater_time = 1 * MIN; //60 * MIN;
long fan_time = 5 * MIN;
long print_time = 5 * SECONDS;
long light_time = 5 * MIN;

/**********************************************************************************************/
/*INT Adafruit Data Shield
 *********************************************************************************************
   SD card attached to SPI bus as follows:
 ** Mega:  MOSI - pin 51, MISO - pin 50, CLK - pin 52, CS - pin 4 (CS pin can be changed)
  and pin #52 (SS) must be an output*/
const int chipSelect = 10; // this is the CS for the data logger
#include <SPI.h>
#include <Wire.h> //This is the library that helps the Arduino with i2c
#include "SD.h" // This is the library for talking to the card
File dataFile;

//******************* I need to read up on this and see if this can go in the setup. I dont want the system to stop if there is something wrong with the data logger.***********
//#define LOG_INTEVAL 1000 //This is how many milli seconds between the sensor readings (i woul like to figure out these features, but currently not using.
//#define ECHO_TO_SERIAL 1 //this is if i want to echo to the serial monitor (1 is on and 0 is off)
void error(char *str)// the error function is uded if there is something bad happening (i.e. no micro SD card is installed. It will sit in the "while(1) loop forever until the issue is corrected
{
  Serial.print("error: ");
  Serial.println(str);

  //while (1);
}

/*********************************************************************************************
   INT Time Stamp
 * ******************************************************************************************/
#include "RTClib.h" //this is the library the Arduino uses to chat with the real time clock
RTC_PCF8523 rtc; // I found a different variable (RTC_DS1307) If the time is incorrect than i want to try this

/***************************************************************************************************
  INT TSL2591 Digital Light Sensor
***************************************************************************************************/
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
unsigned long ir;

/******************************************************************************************
  INT Temp, Humid, Baro BMP280 (outside sensor)
 *******************************************************************************************/
const int BME_SCK = 13;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_MISO = 12;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_MOSI = 11;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_CS = 9; // Chip select for the BMP280 (temp, hum, baro) sensor
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
//Adafruit_BME280 bme(BME_CS); // hardware SPI
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

/*********************************************************************************************
  INT Temp Sensor DS18B20
 *********************************************************************************************/
#include <OneWire.h>
#include <DallasTemperature.h>
const int ONE_WIRE_BUS = 2; // Pin for DS18B20 sensors (shed, outside, cricket, jango)
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with a OneWire device
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor
DeviceAddress outside_temp = { 0x28, 0xF4, 0x45, 0x95, 0xF0, 0x01, 0x3C, 0xBF};// this is the shed sensor in the rafters
DeviceAddress cricket_temp = { 0x28, 0x8A, 0x95, 0xF7, 0x4B, 0x20, 0x1, 0xC6 };// this is cricket
DeviceAddress shed_temp = {0x28, 0xD6, 0xE4, 0x10, 0x4C, 0x20, 0x01, 0x80 };// This is the outside sensor in weather station
DeviceAddress jango_temp = {0x28, 0xE7, 0xDF, 0xE2, 0x4B, 0x20, 0x1, 0xA2};// this is Jango
//DeviceAddress relay_temp = { 0x28, 0x21, 0xB3, 0x95, 0xF0, 0x01, 0x3C, 0xAA };// Need to get the info for this sensor.

/************************************************************************************************
  INT DHT11 temp and humidity sensor
 ***********************************************************************************************/
#include "DHT.h"
#define DHTTYPE DHT11 //type of sensor (may get DHT22 in future)
#define sensor1 3 //Sensor inside of the pi case (~ Pi/Arduino temp)
#define sensor2 4 //Sensor outside of the pi case (~shed temp)
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

  sensors.begin(); // this is for the DS18B20 (oneWire)

  dht1.begin();//DHT11 Temperature Sensors inside pelican case
  dht2.begin();//DHT11 Temperature Sensors outside pelican case

  bme.begin();// BME280 Temp Sensor
  
    /**************************************************************************************************
    SETUP Data Logger Shield Setup
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
  dataFile = SD.open("datalog.csv", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error creating file, SD may be full");
  }
  Serial.print("Logging to: ");
  Serial.println(filename);

  /**************************************************************************************************
    SETUP Data Logger Shield Setup
  *************************************************************************************************/
  Wire.begin(); // i added this, so lets see if it works.
  //    if (! rtc.begin()) {
  //      Serial.println("Couldn't find RTC");
  //      while (1);
  //    }

  dataFile.print("date/time"); // this is going to be the top column of the excel file, so adjust accordingly
  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    //rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }  
  delay(100);// there may need to be a slight delay here for the sensor to 'warm up'
}
/*********************************************************************************************************************
*********************************************************************************************************************
                                LOOP
*********************************************************************************************************************
*********************************************************************************************************************/
void loop() {
  unsigned long currentMillis = millis();

  float hum1 = dht1.readHumidity();
  float hum2 = dht2.readHumidity();
  float tempF1 = dht1.readTemperature(true);//DHT11 Temperature Sensors inside pelican case
  float tempF2 = dht2.readTemperature(true);//DHT11 Temperature Sensors outside pelican case

  //Temp Sensor DS18B20
  sensors.requestTemperatures(); // Send the command to get temperatures
  float cricket_var = sensors.getTempF(cricket_temp);
  float jango_var = sensors.getTempF(jango_temp);
  float outside_var = sensors.getTempC(outside_temp);
  float outsideF_var = sensors.getTempF(outside_temp);
  float shed_var = sensors.getTempF(shed_temp);
  //float relay_var = sensors.getTempF(relay_temp);
  //TODO: I still need to solder this wire into the board

  //BMP280 (outside sensor)
  float ws_temp_var = bme.readTemperature();

  //Variables
  float dk_avg = (cricket_var + jango_var) / 2;
  float shed_avg = (tempF2 + shed_var) / 2;
  float outside_avg = (ws_temp_var + outside_var) / 2;
  
    /**************************************************************************************************
      Light Loop
  **************************************************************************************************/
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  // tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
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

  /***********************************************************************************************************
    Serial Print
  ***********************************************************************************************************/
  if ((currentMillis - print_previousMillis) >= print_time) {
	  
    DateTime now = rtc.now();
    Serial.print(now.year(), DEC); Serial.print('/'); Serial.print(now.month(), DEC); Serial.print('/');
    Serial.print(now.day(), DEC); Serial.print(") "); Serial.print(now.hour(), DEC); Serial.print(':');
    Serial.print(now.minute(), DEC); Serial.print(':'); Serial.println(now.second(), DEC);
    Serial.print("Shed Temp: "); Serial.print(shed_var); Serial.print("   Comp Temp: "); Serial.print(tempF1);
    Serial.print("  Cricket Temp: "); Serial.print(cricket_var); Serial.print("  Jango Temp: "); Serial.print(jango_var);
    //Serial.print("  Relay Temp:"); Serial.print(relay_var);
    Serial.print("  Outside temp:"); Serial.print(outside_var);
    Serial.print("(*C): "); Serial.print(outsideF_var); Serial.println("(*F): ");
	
	Serial.print(" Humidity = "); Serial.print(bme.readHumidity()); Serial.print(" %");Serial.print(F("  Full: ")); Serial.print(full); Serial.println(F("  "));
	
	    Serial.print("Dog Kennel AVG:"); Serial.print(dk_avg); Serial.print("  Outside AVG:"); Serial.print(outside_avg);
    Serial.print("  Shed AVG:"); Serial.println(shed_avg);
	
    /***********************************************************************************************************
//      Data File Print
//    ***********************************************************************************************************/
    dataFile.print(now.year(), DEC); dataFile.print('/'); dataFile.print(now.month(), DEC); dataFile.print('/');
    dataFile.print(now.day(), DEC); dataFile.print(") "); dataFile.print(","); dataFile.print(now.hour(), DEC);
    dataFile.print(':'); dataFile.print(now.minute(), DEC); dataFile.print(':'); dataFile.print(now.second(), DEC);
    dataFile.print(",");

    dataFile.print("Light Sensor: ir full full-ir lux"); dataFile.print(","); dataFile.print(ir); dataFile.print(",");
    dataFile.print(full); dataFile.print(","); dataFile.print(full - ir); dataFile.print(","); dataFile.print(tsl.calculateLux(full, ir), 6);
	
	print_previousMillis = currentMillis;
}
}
