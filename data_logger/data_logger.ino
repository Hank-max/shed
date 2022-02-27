
// A simple data logger for the Arduino analog pins
#define LOG_INTERVAL  1000 // mills between entries (not sure if this is part fo the library. Not going to use. 
#define WAIT_TO_START    0 // Wait for serial input in setup() 0=off and 1=on; 
//if this is on, then you will have to hit any key in the serial monitor to start.

// the digital pins that connect to the LEDs 
//TODO
//#define redLEDpin 3
//#define greenLEDpin 4

// The analog pins that connect to the sensors
//#define photocellPin 0           // analog 0
//#define tempPin 1  

/**********************************************************************************************/
/*INT Adafruit Data Shield and RTS
 *********************************************************************************************/
#include "SD.h"
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 RTC; // define the Real Time Clock object
const int chipSelect = 10;// for the data logging shield, we use digital pin 10 for the SD cs line
File logfile;// the logging file

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);

  //while (1); I have this shut off because i dont want the data shield to stop the Arduino
}

//void setup(void) //Not sure why they use this additional void in the Adafruit example
void setup()
{
  Serial.begin(9600);
  Serial.println();

    /**************************************************************************************************
    SETUP Data Logger Shield Setup and RTS
  *************************************************************************************************/
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);

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
#if ECHO_TO_SERIAL
  Serial.println("millis,time,light,temp");
#if ECHO_TO_SERIAL// attempt to write out the header to the file
  if (logfile.writeError || !logfile.sync()) {
    error("write header");
  }

  //pinMode(redLEDpin, OUTPUT); //TODO: Need to choose something else that is going to be read to the Logger
  //pinMode(greenLEDpin, OUTPUT);

  // If you want to set the aref to something other than 5v
  //analogReference(EXTERNAL);
}

void loop(void)
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
  logfile.print(now.get()); // seconds since 2000
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

  Serial.print(now.get()); // seconds since 2000
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


  //int photocellReading = analogRead(photocellPin);
  delay(10);
  //int tempReading = analogRead(tempPin);

// TODO this is for what ever we are going to print to the SD card.
  logfile.print(", ");
  logfile.print(photocellReading);
  logfile.print(", ");
  logfile.println(temperatureF);

}
