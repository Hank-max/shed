

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

  /***********************************************************************************************************
    Serial Print
  ***********************************************************************************************************/
if ((currentMillis - print_previousMillis) >= print_time) {
    Serial.print("Shed Temp: "); Serial.print(shed_var); Serial.print("   Comp Temp: "); Serial.print(tempF1);
    Serial.print("  Cricket Temp: "); Serial.print(cricket_var); Serial.print("  Jango Temp: "); Serial.print(jango_var);
    //Serial.print("  Relay Temp:"); Serial.print(relay_var);
    Serial.print("  Outside temp:"); Serial.print(outside_var);
    Serial.print("(*C): "); Serial.print(outsideF_var); Serial.println("(*F): ");
	
	print_previousMillis = currentMillis;
}
}
