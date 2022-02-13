

const int dk_lights = 33; // 12V relay to lights in dog kennel.
const int dk_fans = 43; //SS relay for fans in dog kennel.
const int comp_fans = 34;//12V relay for fans in computer box.
const int shed_fan = 47; // SS relay for shed fan.
const int heater = 39; //SS relay for IR Heater.



/**********************************************************************************************
*  Loop Times 
***********************************************************************************************/

const unsigned long SECONDS = 1000;
const unsigned long MIN = (60 * SECONDS);

unsigned long fan_previousMillis = 0;
unsigned long comp_previousMillis = 0;
unsigned long win_previousMillis = 0;
unsigned long heater_previousMillis = 0;
unsigned long print_previousMillis = 0;//this is for serial print


long win_time = 15000;
long compfan_time = 30 * SECONDS;
long heater_time = 1*MIN; //60 * MIN;
long fan_time = 5 * MIN;
long print_time = 5*SECONDS;

/**********************************************************************************************/
/*Adafruit Data Shield
 *********************************************************************************************
   SD card attached to SPI bus as follows:
 ** Mega:  MOSI - pin 51, MISO - pin 50, CLK - pin 52, CS - pin 4 (CS pin can be changed)
  and pin #52 (SS) must be an output*/
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
const int BME_SCK = 13;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_MISO = 12;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_MOSI = 11;// this is the ____ for the BMP280 (temp, hum, baro) sensor
const int BME_CS = 9; // Chip select for the BMP280 (temp, hum, baro) sensor
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
//Adafruit_BME280 bme(BME_CS); // hardware SPI
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

/*********************************************************************************************
   Temp Sensor DS18B20
 *********************************************************************************************/
const int ONE_WIRE_BUS = 2; // Pin for DS18B20 sensors (shed, outside, cricket, jango)
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with a OneWire device
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor
DeviceAddress outside_temp = { 0x28, 0x24, 0x82, 0x95, 0xF0, 0x01, 0x3C, 0x08};// this is the shed sensor in the rafters
DeviceAddress cricket_temp = { 0x28, 0xD2, 0xB9, 0x3E, 0x46, 0x20, 0x01, 0x2A };// this is cricket
DeviceAddress shed_temp = {0x28, 0xD6, 0x45, 0x95, 0xF0, 0x01, 0x3C, 0x67 };// This is the outside sensor in weather station
DeviceAddress jango_temp = {0x28, 0x7E, 0xE2, 0x95, 0xF0, 0x01, 0x3C, 0xF8};// this is Jango
DeviceAddress relay_temp = { 0x28, 0x21, 0xB3, 0x95, 0xF0, 0x01, 0x3C, 0xAA };// Need to get the info for this sensor.

/************************************************************************************************
   DHT11 temp and humidity sensor
 ***********************************************************************************************/
#include "DHT.h"
#define DHTTYPE DHT11 //type of sensor (may get DHT22 in future)
#define sensor1 3 //Sensor inside of the pi case (~ Pi/Arduino temp)
#define sensor2 4 //Sensor outside of the pi case (~shed temp)
DHT dht1(sensor1, DHTTYPE);
DHT dht2(sensor2, DHTTYPE);

/********************************************************************************************************
* Window Motor
*********************************************************************************************************/
#define dirPin 26 //Direction of the window motor CW or CCW
#define stepPin 28 //Signal for window motor Microstep Driver
#define win_relay 36 //12V relay to window motor Microstepper
#define op_switch 25 //open indicator switch for window
#define cl_switch 29 //close indicator switch for window
#define stepsPerRevolution 1600*2 //number of window motor revolutions


/*********************************************************************************************************************
*********************************************************************************************************************
                                SETUP
*********************************************************************************************************************
*********************************************************************************************************************/
void setup()
{
  Serial.begin(9600);
  //Serial.begin(115200);// This has to be 115200 for the DS18B20 sensors to work in series
  dht1.begin();

  pinMode(dk_fans, OUTPUT);
  pinMode(shed_fan, OUTPUT);
  pinMode(comp_fans, OUTPUT);
  pinMode(motor, OUTPUT);
  pinMode(heater, OUTPUT);
  
    pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(win_relay, OUTPUT);
  pinMode(op_switch, INPUT);
  pinMode(cl_switch, INPUT);

  
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
 delay(100);// there may need to be a slight delay here for the sensor to 'warm up'
}

void loop() {
  unsigned long currentMillis = millis();
  
  float hum1 = dht1.readHumidity();
  float tempF1 = dht1.readTemperature(true);// this is the sensor on the bread board
  
  float cricket_var = sensors.getTempF(cricket_temp);
  float jango_var = sensors.getTempF(jango_temp);
  float shed_var = sensors.getTempF(shed_temp);
  float relay_var = sensors.getTempF(relay_temp);
  float outside_var = sensors.getTempC(outside_temp);
  
    int  close_value = digitalRead(cl_switch);
  int open_value = digitalRead(op_switch);
  
  float dk_avg = (cricket_var + jango_var) / 2;
  float shed_avg = (outside_var + shed_var) / 2;
  float outside_avg = (outside_var);

/*
 * All of the fans can be checked at the same interval. 
 */
if (currentMillis - fan_previousMillis >= fan_time)) {
if (outside_avg > 21){
    if (dk_avg > 60) {
      digitalWrite(dk_fans, HIGH);
      Serial.println("   FAN IS ON");
    }
}
    else if (dk_avg <= 60) {
      digitalWrite(dk_fans, LOW);
      Serial.println("   FAN IS OFF");
    }
  else if (outside_avg < 61) {
    if (dk_avg >= 65) {
      digitalWrite(dk_fans, HIGH);
      Serial.println("  FAN IS ON");
    }
    else if (dk_avg < 65) {
      digitalWrite(dk_fans, LOW);
      Serial.println("  FAN IS OFF");
    }
  }
  if (shed_avg >= 70) {
    digitalWrite (shed_fan, HIGH);
  }
  else {
    digitalWrite (shed_fan, LOW);
  }
  fan_previousMillis = currentMillis;
}

/*
 * The computer fan needs to be checked more frequently due to continuous temperature change
 */
  if ((tempF1 >= 65) && (currentMillis - comp_previousMillis <= compfan_time)) {
    digitalWrite (comp_fans, LOW); //The relay requires a low input to trigger the relay
comp_previousMillis = currentMillis;
  }
  else {
    digitalWrite (comp_fans, HIGH);
comp_previousMillis = currentMillis;
  }

/* 
 *  This is the loop for the IR Heater, which is only going to run the first hour of REALLY cold days. 
 */
  if ((outside_avg <= 0) && (dk_avg < 40) && (currentMillis <= (heater_time)) 
  {
  digitalWrite(heater, HIGH);
  digitalWrite(dk_fans, HIGH);
  //Serial.println("Heater");
  //heater_previousMillis = currentMillis;
  //i do not need a previous millis becuase i want this to just run the first hour when the system is turned on. 
  }
  else{
    digitalWrite(heater, LOW);
    digitalWrite(dk_fans, LOW);
  }
   
   /******************************************************************************************
   * Loop for Window Motor
   *******************************************************************************************/
   //NEED TO FIGURE OUT THE TEMPERATURES
     if ((temp### > ##) && (temp### > ##) && (currentMillis - win_previousMillis >= win_time)) {
    digitalWrite(win_relay, LOW);
    delay(100);//this is to give the relay time to turn on/off.
    if ((close_value == HIGH) && (open_value == LOW)) {
      digitalWrite(dirPin, HIGH);// this is setting the direction
      for (int i = 0; i < stepsPerRevolution; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
      }
      win_previousMillis = currentMillis;
      delay(100);
      digitalWrite(win_relay, HIGH);
    }

    else if ((close_value == LOW) && (open_value == HIGH)) {
      digitalWrite(dirPin, LOW);
      delay(100);
      for (int i = 0; i < stepsPerRevolution; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
      }
      win_previousMillis = currentMillis;
      delay(100);
      digitalWrite(win_relay, HIGH);
      Serial.println("   else");
    }
  }

  /***********************************************************************************************************
    Serial Print
  ***********************************************************************************************************/
 if (currentMillis - ser_previousMillis >=  (print_time){
  Serial.print("Shed Temperature: ");
  Serial.print(shed_var);
  Serial.print("   Bread Board Temp: ");
  Serial.print(tempF1);
  Serial.print("  Cricket Temp: ");
  Serial.print(cricket_var);
  Serial.print("  Jango Temp: ");
  Serial.print(jango_var);
  Serial.print("  Relay Temp:");
  Serial.print(relay_var);
  Serial.print("  Outside temp:");
  Serial.println(outside_var); 
  ser_previousMillis = currentMillis;
 }
 
  // delay(5 * MIN);
  //delay(5000);
}
