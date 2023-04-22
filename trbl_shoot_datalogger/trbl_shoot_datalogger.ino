#include <SPI.h>
#include <SD.h>
File dataFile;
#include "RTClib.h"

RTC_PCF8523 rtc;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

#include "DHT.h"
#define DHTTYPE DHT11 //type of sensor (may get DHT22 in future)
#define sensor1 2 //this is the pin location for the DHT11

// this is just code that I stole from Arduino Example for the light sensor, so ignore all of the 'blue' stuff
const int blueSensorPin = A3;
int blueSensorValue = 0;
int blueValue = 0;

DHT dht1(sensor1, DHTTYPE);

const int chipSelect = 10;

void setup(void) {
  Serial.begin(57600);
  //Serial.begin(9600);
  dht1.begin();

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

  Serial.print("Offset is "); Serial.println(offset); // Print to control offset

  pinMode(SS, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("SDCard failed, or not present");
    // don't do anything more:
    while (1) ;
  }
  Serial.println("card initialized.");

  // Open up the file we're going to log to!
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error opening datalog.txt");
    // Wait forever since we cant write data
    while (1) ;
  }
}

void loop(void) {

  blueSensorValue = analogRead(blueSensorPin);
  blueValue = blueSensorValue / 4;

  //DHT11 Temperature Sensors
  float hum1 = dht1.readHumidity();
  float tempF1 = dht1.readTemperature(true);
  float tempC1 = dht1.readTemperature();

  DateTime now = rtc.now();
  String dataString = "";
  dataString += String(now.year(), DEC); dataString += String(now.month(), DEC); dataString += String(now.day(), DEC);
  dataString += ","; dataString += String(now.hour(), DEC); dataString += ":"; dataString += String(now.minute(), DEC);
  dataString += ":"; dataString += String(now.second(), DEC); dataString += ",";
  dataString += String(hum1); dataString += ","; dataString += String(tempF1); dataString += ",";
  dataString += String(tempC1); dataString += ","; dataString += String(blueValue); dataString += ",";
  dataString += String(blueSensorValue);
  dataFile.println(dataString);

  dataFile.flush();



  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  Serial.println(dataString);
  Serial.print(" Humidity: "); Serial.println(hum1);
  Serial.print(" Temperature F: "); Serial.println(tempF1);
  Serial.print(" Temperature C: "); Serial.println(tempC1);
  Serial.print(" Light: "); Serial.println(blueValue);
  Serial.print(" Light Raw: "); Serial.println(blueSensorValue);
  delay(30000);

}
