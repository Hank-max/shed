
#include "DHT.h"

#define sensor1 2 // Digital pin connected to the DHT sensor
#define sensor2 3
#define sensor3 4
#define light 6
#define relay 7
const unsigned long Second = 1000;
const unsigned long Min = 60 * Second;//Did this to make changing the delay easier

// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.
#define DHTTYPE DHT11   // DHT 11
// Initialize DHT sensor.
DHT dht1(sensor1, DHTTYPE);
DHT dht2(sensor2, DHTTYPE);
DHT dht3(sensor3, DHTTYPE);


void setup() {
  Serial.begin(9600);
  dht1.begin();
  dht2.begin();
  dht3.begin();
  pinMode(relay, OUTPUT);
  pinMode(light, OUTPUT);

}
void loop() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  delay(2000);

//************************************** DHT 11 SETUP ****************************************
  float hum1 = dht1.readHumidity();
  float hum2 = dht2.readHumidity();
  float hum3 = dht3.readHumidity();
  // Read temperature as Celsius (the default)
  //  float tempC1 = dht1.readTemperature();
  //  float tempC2 = dht2.readTemperature();
  //  float tempC3 = dht3.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float tempF1 = dht1.readTemperature(true);
  float tempF2 = dht2.readTemperature(true);
  float tempF3 = dht3.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(hum1) || isnan(tempF1) || isnan(hum2) || isnan(tempF2) || isnan(hum3) || isnan(tempF3)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    //I also want to put this on the LED once i get it up and running
    return;
  }
  // Compute heat index in Fahrenheit (the default)
  // float hif = dht1.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  // float hic = dht1.computeHeatIndex(t, h, false);

  int n = 3; //this is the number of sensors

  //************************************************ CHECKING THE TEMPERATURE ***************************************
  float tempArray [n] = {tempF1, tempF2, tempF3};
  float maxTempF = tempArray[0];
  float minTempF = tempArray[0];
  float avgTemp = (tempF1 + tempF2 + tempF3) / n;
  Serial.println("************ TEMP *******************");

  for (int i = 0; i < n; i++)
  {
    maxTempF = max(tempArray[i], maxTempF);
    minTempF = min(tempArray[i], minTempF);
    Serial.println(tempArray[i]);
  }
  Serial.print ("Max Temp: "); Serial.print(maxTempF); Serial.print("  Min Temp  "); Serial.println(minTempF);

  if ((maxTempF - abs(minTempF)) > 5)// This is is to solve for the event of a faulty sensor
  {
    if (maxTempF > (5 + avgTemp)) {
      avgTemp = (tempF1 + tempF2 + tempF3 - maxTempF) / (n - 1);
    }
    else
      avgTemp = (tempF1 + tempF2 + tempF3 - minTempF) / (n - 1);
  }
  Serial.print("average temp: ");
  Serial.println(avgTemp);

  //********************************************* CHECKING THE HUMIDITY *********************************
  float humArray [n] = {hum1, hum2, hum3};
  float maxHum = humArray[0];
  float minHum = humArray[0];
  float  avgHum = (hum1 + hum2 + hum3) / n;
  Serial.println("************ Humidity *******************");

  for (int j = 0; j < n; j++)
  {
    maxHum = max(humArray[j], maxHum);
    minHum = min(humArray[j], minHum);
    Serial.println(humArray[j]);
  }
  Serial.print ("Max Hum: "); Serial.print(maxHum); Serial.print("  Min Hum  "); Serial.println(minHum);

  if ((maxHum - abs(minHum)) > 10)// This is is to solve for the event of a faulty sensor
  {
    if (maxHum > (10 + avgHum)) {
      avgHum = (hum1 + hum2 + hum3 - maxHum) / (n - 1);
    }
    else
      avgHum = (hum1 + hum2 + hum3 - minHum) / (n - 1);
  }
  Serial.print("average Hum: ");
  Serial.println(avgHum);

//***************************************** TURNING ON THE FAN ***********************************************

  if (avgHum >= 50) {
    digitalWrite(relay, HIGH);
    analogWrite(light,(20));//so when the fan is on the light turns on. I did analogWrite because i felt the LED was to bright
  }
  else {
    digitalWrite(relay, LOW);
    digitalWrite(light, LOW);
  }
  // Wait a few minutes between measurements so the fan is not always turning on and off.
  delay(10*Min);

}
