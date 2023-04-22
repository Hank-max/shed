

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

long win_time = 10 * MIN;//this time needs to be >fan_time because of the window var
long compfan_time = 30 * SECONDS;
long heater_time = 30 * MIN; //60 * MIN;
long fan_time = 5 * MIN;//this is actually for the whole temp loop
long print_time = 3 * MIN;
long light_time = MIN;
long startup_time = 2 * SECONDS;//loop time for the items turned on @ start up.

/**********************************************************************************************/
/*INT Adafruit Data Shield and RTS
 *********************************************************************************************
   SD card attached to SPI bus as follows:
 ** Mega:  MOSI - pin 51, MISO - pin 50, CLK - pin 52, CS - pin 4 (CS pin can be changed)
  and pin #52 (SS) must be an output*/
const int chipSelect = 10; // this is the CS for the data logger
#include <SPI.h>
#include <Wire.h> //This is the library that helps the Arduino with i2c
#include <SD.h> // This is the library for talking to the card
File dataFile;
#include "RTClib.h" //this is the library the Arduino uses to chat with the real time clock
RTC_PCF8523 rtc; // I found a different variable (RTC_DS1307) If the time is incorrect than i want to try this
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


//******************* I need to read up on this and see if this can go in the setup. I dont want the system to stop if there is something wrong with the data logger.***********
//#define LOG_INTEVAL 1000 //This is how many milli seconds between the sensor readings (i woul like to figure out these features, but currently not using.
//#define ECHO_TO_SERIAL 1 //this is if i want to echo to the serial monitor (1 is on and 0 is off)
void error(char *str)// the error function is uded if there is something bad happening (i.e. no micro SD card is installed. It will sit in the "while(1) loop forever until the issue is corrected
{
  Serial.print("error: Prob No Micro SD Card Installed");
  Serial.println(str);

  //while (1);
}


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
DeviceAddress relay_temp = { 0x28, 0xD2, 0xB9, 0x3E, 0x46, 0x20, 0x01, 0x2A };// Need to get the info for this sensor.

/************************************************************************************************
  INT DHT11 temp and humidity sensor
 ***********************************************************************************************/
#include "DHT.h"
#define DHTTYPE DHT11 //type of sensor (may get DHT22 in future)
#define sensor1 3 //Sensor inside of the pi case (~ Pi/Arduino temp)
#define sensor2 4 //Sensor outside of the pi case (~shed temp)
DHT dht1(sensor1, DHTTYPE);
DHT dht2(sensor2, DHTTYPE);

/********************************************************************************************************
  INT Window Motor
*********************************************************************************************************/
#define dirPin 26 //Direction of the window motor CW or CCW
#define stepPin 28 //Signal for window motor Microstep Driver
#define win_relay 36 //12V relay to window motor Microstepper
#define op_switch 29 //open indicator switch for window might have these backwards.
#define cl_switch 25 //close indicator switch for window
#define motor_comm_pwr 35 //the digitial pin giving COMM power to the switches
#define stepsPerRevolution 1600*2
int window_var; //Variable based on temperature for window
int op_lastButtonState;   //Open Switch Debounce
int cl_lastButtonState;   //Close Switch Debounce
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time

/*********************************************************************************************************************
*********************************************************************************************************************
                                SETUP
*********************************************************************************************************************
*********************************************************************************************************************/
void setup()
{
  //Serial.begin(9600);
    Serial.begin(57600);
  // Serial.begin(115200);

  sensors.begin(); // this is for the DS18B20 (oneWire)

  dht1.begin();//DHT11 Temperature Sensors inside pelican case
  dht2.begin();//DHT11 Temperature Sensors outside pelican case

  bme.begin();// BME280 Temp Sensor


  pinMode(dk_fans, OUTPUT);
  pinMode(shed_fan, OUTPUT);
  pinMode(comp_fans, OUTPUT);
  pinMode(heater, OUTPUT);
  pinMode(dk_lights, OUTPUT);

  digitalWrite(comp_fans, HIGH);//turning the relay off for startup
  digitalWrite(win_relay, HIGH);//turning the relay off for startup

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(win_relay, OUTPUT);
  pinMode(op_switch, INPUT);
  pinMode(cl_switch, INPUT);
  pinMode(motor_comm_pwr, OUTPUT);

int window_var = 0;
  /**************************************************************************************************
    SETUP Data Logger Shield Setup and RTS
  *************************************************************************************************/
 
#ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
#endif

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  rtc.start();

  float drift = 43; // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
  float period_sec = (7 * 86400);  // total obsevation period in seconds (86400 = seconds in 1 day:  7 days = (7 * 86400) seconds )
  float deviation_ppm = (drift / period_sec * 1000000); //  deviation in parts per million (μs)
  float drift_unit = 4.34; // use with offset mode PCF8523_TwoHours
  // float drift_unit = 4.069; //For corrections every min the drift_unit is 4.069 ppm (use with offset mode PCF8523_OneMinute)
  int offset = round(deviation_ppm / drift_unit);
  // rtc.calibrate(PCF8523_TwoHours, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
  // rtc.calibrate(PCF8523_TwoHours, 0); // Un-comment to cancel previous calibration

  pinMode(SS, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("SDCard failed, or not present");
    // don't do anything more:
   // while (1) ;
  }
  Serial.println("card initialized.");

  // Open up the file we're going to log to!
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error opening datalog.txt");
    // Wait forever since we cant write data
    //while (1) ;
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
  digitalWrite(motor_comm_pwr, HIGH);//Continuous 5V signal to window switches (COMM)

  //DHT11 Temperature Sensors
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
  float relay_var = sensors.getTempF(relay_temp);

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

  if (currentMillis - light_previousMillis >= light_time) {
    if (full < 5000) {
      digitalWrite(dk_lights, LOW);//the relay requires a low signal to trigger the relay
      //Serial.print( " LIGHT IS ON  ");
    }
    else {
      digitalWrite(dk_lights, HIGH);
      //Serial.print( " LIGHT IS OFF  ");
    }
    light_previousMillis = currentMillis;
  }
  /****************************************************************************************************
     Fans Loop and Temperature Variables
   ****************************************************************************************************/
  if ((currentMillis - fan_previousMillis >= fan_time)) {
    if (outside_avg > 16) {
      if (dk_avg > 60) {
        digitalWrite(dk_fans, HIGH);
        window_var = 2;
        //Serial.print("   DK FAN IS ON  ");
      }
      else if ((dk_avg <= 60) && (dk_avg > 50)) {
        digitalWrite(dk_fans, LOW);
        window_var = 2;
        //Serial.print("   DK FAN IS OFF  ");
      }
      else {
        //If the DK is colder than 50 and it is nice out
        digitalWrite(dk_fans, LOW);
        window_var = 1;
        //Serial.print("   DK FAN IS OFF  ");
      }
    }
    else if (outside_avg <= 16) {
      if (dk_avg >= 70) {
        digitalWrite(dk_fans, HIGH);
        window_var = 1;
        //Serial.print("  DK FAN IS ON  ");
      }
      else if ((dk_avg < 70) && (dk_avg > 62)) {
        digitalWrite(dk_fans, HIGH);
        window_var = 2;
        //Serial.print("  DK FAN IS ON  ");
      }
      else {
        digitalWrite(dk_fans, LOW);
        window_var = 2;
        //Serial.print("  DK FAN IS OFF  ");
      }
    }
    //Shed Fan Loop
    if (shed_avg >= 70) {
      digitalWrite (shed_fan, HIGH);
      //Serial.print("   Shed FAN IS ON  ");
    }
    else {
      digitalWrite (shed_fan, LOW);
      //Serial.print("   Shed FAN IS OFF  ");
    }
    fan_previousMillis = currentMillis;
  }

  /*******************************************************************************************************
     Computer Fan Loop
   *******************************************************************************************************/
  if (currentMillis - comp_previousMillis >= compfan_time) {
    if (tempF1 >= 70) {
      digitalWrite (comp_fans, LOW); //The relay requires a low input to trigger the relay
      //Serial.print("   Comp FAN IS ON  ");
    }
    else {
      digitalWrite (comp_fans, HIGH);
      //Serial.print("   Comp FAN IS OFF  ");
    }
    comp_previousMillis = currentMillis;
  }

  /*******************************************************************************************************
      IR Heater Loop
  ********************************************************************************************************/
  if ((currentMillis <= startup_time) && (dk_avg < 47)) {
    digitalWrite(heater, HIGH);// turning the heater on @ start up.
    Serial.print("  Startup Heater IS ON  ");
  }
  if (currentMillis - heater_previousMillis >= heater_time) {
    if ((outside_avg <= 0) && (dk_avg < 45)) {
      digitalWrite(heater, HIGH);
      //Serial.print("   Heater IS ON  ");
    }
    else {
      digitalWrite(heater, LOW);
      //Serial.print("   Heater IS OFF  ");
    }
    heater_previousMillis = currentMillis;
  }
  /******************************************************************************************
    Loop for Window Motor
  *******************************************************************************************/
  int  close_value = digitalRead(cl_switch);
  int open_value = digitalRead(op_switch);
  delay(100);

  /*************************************
     Window going from CLOSE to OPEN
  **************************************/
  if ((window_var == 1) && (currentMillis - win_previousMillis >= win_time)) {
    op_lastButtonState = LOW;
    if ( open_value != op_lastButtonState) {
      lastDebounceTime = currentMillis;
    }
    if ((open_value == LOW) && (close_value == HIGH) && ((currentMillis - lastDebounceTime) > debounceDelay)) {
      digitalWrite(win_relay, LOW);//turning ON the 12V power to the motor
      digitalWrite(dirPin, HIGH);// this is setting the direction
      delay(100);//this is to give the relay time to turn on/off.

      while (open_value == LOW) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
        open_value = digitalRead(op_switch);
      }
      win_previousMillis = currentMillis;
      delay(100);
      //Serial.println("   Window IS OPEN  ");
      digitalWrite(win_relay, HIGH);//turning off the 12V power to the motor
      window_var = 0;
    }
  }

  /************************************
     Window going from OPEN to CLOSE
  *************************************/
  if ((window_var == 2) && (currentMillis - win_previousMillis >= win_time)) {
    cl_lastButtonState = LOW;
    if ( close_value != cl_lastButtonState) {
      lastDebounceTime = currentMillis;
    }
    if ((open_value == HIGH) && (close_value == LOW) && ((currentMillis - lastDebounceTime) > debounceDelay)) {
      digitalWrite(win_relay, LOW);//turning ON the 12V power to the motor
      digitalWrite(dirPin, LOW);// this is setting the direction
      delay(100);//this is to give the relay time to turn on/off.

      while (close_value == LOW) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
        close_value = digitalRead(cl_switch);
      }
      win_previousMillis = currentMillis;
      delay(100);
      //Serial.println("   Window IS CLOSED  ");
      digitalWrite(win_relay, HIGH);//turning off the 12V power to the motor
      window_var = 0;
    }
  }
  /***********************************************************************************************************
    Serial Print
  ***********************************************************************************************************/
  if ((currentMillis - print_previousMillis) >= print_time) {

    DateTime now = rtc.now();

  Serial.print(now.year(), DEC);Serial.print('/');Serial.print(now.month(), DEC);Serial.print('/');
  Serial.print(now.day(), DEC);Serial.print(" (");Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);Serial.print(") ");
  Serial.print(now.hour(), DEC);Serial.print(':');Serial.print(now.minute(), DEC);Serial.print(':');Serial.print(now.second(), DEC);
  Serial.println();

    Serial.print(" Low active relays (1=OFF & 0=ON): ");
    Serial.print(" comp_fans="); Serial.print(digitalRead(comp_fans)); Serial.print("  dk_lights="); Serial.print(digitalRead(dk_lights)); Serial.print("  win_relay="); Serial.println(digitalRead(win_relay));
    Serial.print(" High active relays (0=OFF & 1=ON): ");
    Serial.print("dk_fans="); Serial.print(digitalRead(dk_fans)); Serial.print("  shed_fan="); Serial.print(digitalRead(shed_fan)); Serial.print(" heater="); Serial.println(digitalRead(heater));
Serial.print("Window Var= "); Serial.print(window_var); Serial.print("  open status= "); Serial.print(open_value); Serial.print(" close status= "); Serial.println(close_value);

    Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  ")); Serial.print(" Humidity = "); Serial.print(bme.readHumidity()); Serial.println(" %");

    Serial.print("Shed Temp: "); Serial.print(shed_var); Serial.print("   Comp Temp: "); Serial.print(tempF1);
    Serial.print("  Cricket Temp: "); Serial.print(cricket_var); Serial.print("  Jango Temp: "); Serial.print(jango_var);
    Serial.print("  Relay Temp:"); Serial.print(relay_var);
    Serial.print("  Outside temp:"); Serial.print(outside_var);
    Serial.print("(*C): "); Serial.print(outsideF_var); Serial.println("(*F): ");

    Serial.print("Dog Kennel AVG: "); Serial.print(dk_avg); Serial.print("  Outside AVG: "); Serial.print(outside_avg);
    Serial.print("  Shed AVG: "); Serial.println(shed_avg);

    /***********************************************************************************************************
      //      Data File Print
      //    ***********************************************************************************************************/
    
  String dataString = "";
  dataString += String(now.year(), DEC); dataString += String(now.month(), DEC); dataString += String(now.day(), DEC);
  dataString += ","; dataString += String(now.hour(), DEC); dataString += ":"; dataString += String(now.minute(), DEC);
  dataString += ":"; dataString += String(now.second(), DEC); dataString += ",";
  
  dataString += String(comp_fans); dataString += ","; dataString += String(dk_lights); dataString += ",";dataString += String(win_relay);
  dataString += ","; dataString += String(dk_fans); dataString += ",";dataString += String(shed_fan);dataString += ",";dataString += String(heater);
    dataString += ","; dataString += String(window_var); dataString += ",";dataString += String(open_value);dataString += ",";dataString += String(close_value);
	dataString += ","; dataString += String(shed_var); dataString += ",";dataString += String(tempF1);dataString += ",";dataString += String(cricket_var);
	dataString += ","; dataString += String(jango_var); dataString += ",";dataString += String(relay_var);dataString += ",";dataString += String(outside_var);
	dataString += ","; dataString += String(outsideF_var); dataString += ",";dataString += String(dk_avg);dataString += ",";dataString += String(outside_avg);
	dataString += ",";dataString += String(shed_avg);
  dataFile.println(dataString);

  dataFile.flush();
	

    dataFile.print("Light Sensor: ir full full-ir lux"); dataFile.print(","); dataFile.print(ir); dataFile.print(",");
    dataFile.print(full); dataFile.print(","); dataFile.print(full - ir); dataFile.print(","); dataFile.print(tsl.calculateLux(full, ir), 6);

    print_previousMillis = currentMillis;
  }
}
