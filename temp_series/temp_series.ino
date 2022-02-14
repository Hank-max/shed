
// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is connected to GPIO15
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

DeviceAddress outside_temp = { 0x28, 0xF4, 0x45, 0x95, 0xF0, 0x01, 0x3C, 0xBF};// this is the shed sensor in the rafters
DeviceAddress cricket_temp = { 0x28, 0x8A, 0x95, 0xF7, 0x4B, 0x20, 0x1, 0xC6 };// this is cricket
DeviceAddress shed_temp = {0x28, 0xD6, 0xE4, 0x10, 0x4C, 0x20, 0x01, 0x80 };// This is the outside sensor in weather station
DeviceAddress jango_temp = {0x28, 0xE7, 0xDF, 0xE2, 0x4B, 0x20, 0x1, 0xA2};// this is Jango

void setup(void){
  Serial.begin(115200);
  sensors.begin();
}

void loop(void){ 
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  
  Serial.print("Shed Temperature(*C): ");
  Serial.print(sensors.getTempC(shed_temp)); 
  Serial.print("(*F): ");
  Serial.println(sensors.getTempF(shed_temp)); 
 
  Serial.print("Cricket Temperature(*C): ");
  Serial.print(sensors.getTempC(cricket_temp)); 
  Serial.print("(*F): ");
  Serial.println(sensors.getTempF(cricket_temp)); 
  
  Serial.print("Outside Temperature(*C): ");
  Serial.print(sensors.getTempC(outside_temp)); 
  Serial.print("(*F): ");
  Serial.println(sensors.getTempF(outside_temp)); 

  Serial.print("Jango Temperature (*C): ");
  Serial.print(sensors.getTempC(jango_temp)); 
  Serial.print("(*F): ");
  Serial.println(sensors.getTempF(jango_temp));
  
  delay(2000);
}