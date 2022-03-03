


const int dk_lights = 33; // this is the pin for the 12V relay that is contorlling the lights in the dog kennel.
const int dk_fans = 43; //this is the pin on the board where the SS relay is attached for the fans in the dog kennel
const int comp_fans = 34;// this is the 12V relay that controls the fans in the computer box
const int shed_fan = 47; //This is the fan for the shed
const int chipSelect = 10; // this is the CS for the data logger, which is currently not being used.
const int ONE_WIRE_BUS = 2; // this is the one wire (pin) that is used to read the all of the DS18B20 sensors (shed, outside, cricket, jango)
const int BME_SCK = 13;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_MISO = 12;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_MOSI = 11;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_CS = 9; // this is the chip select for the BMP280 (temp, hum, baro) sensor
#define sensor1 3 //this is the sensor inside of the pi case
#define sensor2 4 //this is the sensor outside of the pi case, so the shed temp
const unsigned long SECONDS = 1000;
const unsigned long MIN = (60 * SECONDS);

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
   Time Stamp
 * ******************************************************************************************/
#include "RTClib.h" //this is the library the Arduino uses to chat with the real time clock
RTC_PCF8523 rtc; // I found a different variable (RTC_DS1307) If the time is incorrect than i want to try this

/***************************************************************************************************
  TSL2591 Digital Light Sensor (this is for shutting off the light inside the dog kennel)
***************************************************************************************************/

#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
unsigned long ir;

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
DeviceAddress outside_temp = { 0x28, 0xF4, 0x45, 0x95, 0xF0, 0x01, 0x3C, 0xBF};// this is the shed sensor in the rafters
DeviceAddress cricket_temp = { 0x28, 0x8A, 0x95, 0xF7, 0x4B, 0x20, 0x1, 0xC6 };// this is cricket
DeviceAddress shed_temp = {0x28, 0xD6, 0xE4, 0x10, 0x4C, 0x20, 0x01, 0x80 };// This is the outside sensor in weather station
DeviceAddress jango_temp = {0x28, 0xE7, 0xDF, 0xE2, 0x4B, 0x20, 0x1, 0xA2};// this is Jango

/************************************************************************************************
   DHT11 temp and humidity sensor
 ***********************************************************************************************/
#include "DHT.h"
#define DHTTYPE DHT11 //leaving this here because i may get the DHT22
DHT dht1(sensor1, DHTTYPE);//this is the sensor inside of the pi case
DHT dht2(sensor2, DHTTYPE);//this is the sensor outside of the pi case, so the shed temp

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
}
  /***********************************************************************
     Time Stamp
   * ********************************************************************/
  void timeStamp () {
    // while (!Serial) {
    //delay(1);
    //}

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
    dataFile.print(",");

  }
  /**************************************************************************
      Light Sensor TSL2591
       Configures the gain and integration time if required
  **************************************************************************/
  void advancedRead() {
//    if (tsl.begin())
//    {
//      Serial.println(F("Found a TSL2591 sensor"));
//    }
//    else
//    {
//      Serial.println(F("No sensor found ... check your wiring?"));
//      while (1);
//    }
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

    pinMode(dk_lights, OUTPUT);//the relay that controls the light
    // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
    // That way you can do whatever math and comparisons you want!
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir, full;
    ir = lum >> 16;
    full = lum & 0xFFFF;

       Serial.print(F("Light Sensor TSL2591"));
    //Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] ")); //this is the time in millis when the measurement was taken, but i dont need because i have the time.
    Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
    Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
    Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
    Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);

    dataFile.print("Light Sensor: ir full full-ir lux");
    dataFile.print(",");
    dataFile.print(ir);
    dataFile.print(",");
    dataFile.print(full);
    dataFile.print(",");
    dataFile.print(full - ir);//need to figure out what this is for.
    dataFile.print(",");
    dataFile.print(tsl.calculateLux(full, ir), 6);
    
    if (full < 5000) {
      digitalWrite(dk_lights, LOW);//the relay requires a low signal to trigger the relay
     Serial.print( " LIGHT IS ON ");
    }
    else {
      digitalWrite(dk_lights, HIGH);
      Serial.print( " LIGHT IS OFF ");
    }

  }

  /**************************************************************************
   *************************************************************************
      All of the Temperature Sensors
  **************************************************************************
  **************************************************************************/
  void Temps() {
    //delay(1000);// there may need to be a slight delay here for the sensor

    /********************************************
       BME280 temp sensor (Weather Station)
      ********************************************/
    unsigned status;
    status = bme.begin();// default settings
//    if (!status) {
//      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
//      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
//      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
//      while (1) delay(10);
//    }


    /**************************************************************************************************
       DS18B20 Temperature Sensors (OneWire) (Most Common Sensor)
     *************************************************************************************************/
    sensors.begin(); // this is for the DS18B20
    sensors.requestTemperatures(); // Send the command to get temperatures

    /**************************************************************************************************
      DHT11 Temperature Sensors (inside of the shed and computer case)
    *************************************************************************************************/
    dht1.begin();
    dht2.begin();
    float hum1 = dht1.readHumidity();
    float hum2 = dht2.readHumidity();
    float tempF1 = dht1.readTemperature(true);// this is the sensor inside the pi case
    float tempF2 = dht2.readTemperature(true);// this is the sensor outside of the pi case, so the shed temp

    /**************************************************************************************************
      Using the temperature data (may want to use matrix to get rid of default sensors)
    *************************************************************************************************/
    float cricket_var = sensors.getTempF(cricket_temp);
    float jango_var = sensors.getTempF(jango_temp);
    float outside_var = sensors.getTempC(outside_temp);
    float shed_var = sensors.getTempF(shed_temp);
    float outsideF_var = sensors.getTempF(outside_temp);
	
	float ws_temp_var = bme.readTemperature();
    float comp_shed_var = dht2.readTemperature(true);

    pinMode(shed_fan, OUTPUT);
    pinMode(dk_fans, OUTPUT);
    pinMode(comp_fans, OUTPUT);
    float dk_avg = (cricket_var + jango_var) / 2;
    float outside_avg = ws_temp_var;//(outside_var + ws_temp_var) / 2; (Just doing this for now till i fix the oneWire sensors)
    float shed_avg = comp_shed_var;//(comp_shed_var + shed_var) / 2;
    
    //this is turning the dog kennel fan on and off based on the temperature outside and in the kennel (outside temp is in C)
    if (outside_avg >= 21) {
      if (dk_avg > 60) {
        digitalWrite(dk_fans, HIGH);
        Serial.println("   FAN IS ON");
      }
      else if (dk_avg <= 60) {
        digitalWrite(dk_fans, LOW);
        Serial.println("   FAN IS OFF");
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
    //i would like to put the line of code that is below inside of the shed temp loop.

    if (tempF1 >= 65) {
      digitalWrite (comp_fans,LOW);//The relay requires a low input to trigger the relay
    }
    else {
      digitalWrite (comp_fans,HIGH);
    }

    //this is for turning on the shed fan
    if (shed_avg >= 70) {
      digitalWrite (shed_fan, HIGH);
    }
    else {
      digitalWrite (shed_fan, LOW);
    }
    /***********************************************************************************************************
      Serial Print
    ***********************************************************************************************************/

    Serial.print(" Temperature (BME280) = ");
    Serial.print(ws_temp_var);
    Serial.print(" *C");
//    Serial.print(" Pressure = ");
//    Serial.print(bme.readPressure() / 100.0F);
//    Serial.print(" hPa");
//    Serial.print(" Approx. Altitude = ");
//    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
//    Serial.print(" m");
    Serial.print(" Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.print("Shed Temperature: ");
    //Serial.print(sensors.getTempC(shed_temp));
    //Serial.print("(*C): ");
    Serial.print(shed_var);
    Serial.print("(*F):  ");
    Serial.print(tempF2);
    Serial.print("   Computer Temp: ");
    Serial.print(tempF1);
    Serial.print("  Cricket Temp: ");
    //Serial.print(sensors.getTempC(cricket_temp));
    //Serial.print("(*C): ");
    Serial.print(cricket_var);
    Serial.print("(*F): ");
    Serial.print("  Outside Temp: ");
    Serial.print(outside_var);
    Serial.print("(*C): ");
    Serial.print(outsideF_var);
    Serial.print("(*F): ");
    Serial.print("  Jango Temp: ");
    //Serial.print(sensors.getTempC(jango_temp));
    //Serial.print("(*C): ");
    Serial.print(jango_var);
    Serial.println("(*F): ");

    Serial.print("Dog Kennel AVG:");
    Serial.print(dk_avg);
    Serial.print("  Outside AVG:");
    Serial.print(outside_avg);
    Serial.print("  Shed AVG:");
    Serial.println(shed_avg);

    /**********************************************************************************************************
      print to data logger
    **********************************************************************************************************/



  }

  void loop() {

    //Logger();
    timeStamp();
    advancedRead();//this is for the light sensor
    Temps();





    // The following line will 'save' the file to the SD card after every
    // line of data - this will use more power and slow down how much data
    // you can read but it's safer!
    // If you want to speed up the system, remove the call to flush() and it
    // will save the file only every 512 bytes - every time a sector on the
    // SD card is filled with data.
    dataFile.flush();

    delay(5 * MIN);
    //delay(5000);
  }
