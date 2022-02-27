#include "SD.h"
#include <Wire.h>
#include "RTClib.h"

// A simple data logger for the Arduino analog pins
#define LOG_INTERVAL  1000 // mills between entries
#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    1 // Wait for serial input in setup()


// the digital pins that connect to the LEDs
#define redLEDpin 3
#define greenLEDpin 4

// The analog pins that connect to the sensors
#define photocellPin 0           // analog 0
#define tempPin 1                // analog 1

RTC_DS1307 RTC; // define the Real Time Clock object


// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while (1);
}

void setup()
{
  Serial.begin(9600);
  Serial.println();

#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }

  if (! logfile) {
    error("couldnt create file");
  }

  Serial.print("Logging to: ");
  Serial.println(filename);

  Wire.begin();
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }


  logfile.println("millis,time,light,temp");

  Serial.println("millis,time,light,temp");


  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);

  // If you want to set the aref to something other than 5v
  //analogReference(EXTERNAL);
}

void loop()
{
  DateTime now;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));

  digitalWrite(greenLEDpin, HIGH);

  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");
#endif

  // fetch the time
  now = RTC.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 2000
  logfile.print(", ");
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);

  Serial.print(now.unixtime()); // seconds since 2000
  Serial.print(", ");
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);


  int photocellReading = analogRead(photocellPin);
  delay(10);
  int tempReading = analogRead(tempPin);

  // converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = (tempReading * 5.0) / 1024.0;
  float temperatureC = (voltage - 0.5) * 100.0 ;
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

  logfile.print(", ");
  logfile.print(photocellReading);
  logfile.print(", ");
  logfile.println(temperatureF);
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(photocellReading);
  Serial.print(", ");
  Serial.println(temperatureF);
#endif //ECHO_TO_SERIAL

  digitalWrite(greenLEDpin, LOW);
}
