
// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is connected to GPIO15
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

//DeviceAddress shed_temp = { 0x28, 0xD6, 0xE4, 0x10, 0x4C, 0x20, 0x01, 0x80 };// this is the shed
//DeviceAddress cricket_temp = { 0x28, 0x8A, 0x95, 0xF7, 0x4B, 0x20, 0x1, 0xC6 };// this is cricket
DeviceAddress outside_temp = { 0x28, 0x71, 0x8, 0x10, 0x4C, 0x20, 0x1, 0xA2 };// This is the outside sensor
//DeviceAddress jango_temp = {0x28, 0xE7, 0xDF, 0xE2, 0x4B, 0x20, 0x1, 0xA2};// this is Jango

#define MAXREADING 10
int readings[MAXREADING];
int index;
bool arraysAreReady;

int calculateAverage(int*readings)
{
  long sum = 0UL;
  if (arraysAreReady) {
    for (byte n = 0; n < MAXREADING; n++) sum += readings[n];
    return sum / MAXREADING;
  }
  else {
    if (index) {
      for (byte n = 0; n < index; n++) sum += readings[n];
      return sum / index;
    }
    else {
      return -1;
    }
  }
}

void displayAverage()
{
  Serial.print("Index is =   ");
  Serial.print(index);
  Serial.print("  -  Average is =  ");
  Serial.println(calculateAverage(readings));
}

void readTempSensor()
{
  sensors.begin();
  sensors.requestTemperatures(); // Send the command to get temperatures
  readings[index] = outside_temp;
  if (++index >= MAXREADING) {
    arraysAreReady = true;
    index = 0;
  }
}

void setup(void) {
  Serial.begin(9600);
  index = 0;
  arraysAreReady = false;
}

void loop(void) {
  //Serial.print("Requesting temperatures...");
  //Serial.println("DONE");
  displayAverage();
  readTempSensor();


  //  Serial.print("Shed Temperature(*C): ");
  //  Serial.print(sensors.getTempC(shed_temp));
  //  Serial.print("(*F): ");
  //  Serial.println(sensors.getTempF(shed_temp));

  //  Serial.print("Cricket Temperature(*C): ");
  //  Serial.print(sensors.getTempC(cricket_temp));
  //  Serial.print("(*F): ");
  //  Serial.println(sensors.getTempF(cricket_temp));

  //  Serial.print("Outside Temperature(*C): ");
  //  Serial.print(sensors.getTempC(outside_temp));
  //  Serial.print("(*F): ");
  //  Serial.println(sensors.getTempF(outside_temp));
  //
  //  Serial.print("Jango Temperature (*C): ");
  //  Serial.print(sensors.getTempC(jango_temp));
  //  Serial.print("(*F): ");
  //  Serial.println(sensors.getTempF(jango_temp));

  delay(5000);

}
