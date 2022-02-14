


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


void setup()
{
  Serial.begin(9600);

 sensors.begin(); // this is for the DS18B20 (oneWire)

void loop() {
  //Temp Sensor DS18B20
  sensors.requestTemperatures(); // Send the command to get temperatures
  float cricket_var = sensors.getTempF(cricket_temp);
  float jango_var = sensors.getTempF(jango_temp);
  float outside_var = sensors.getTempC(outside_temp);
  float outsideF_var = sensors.getTempF(outside_temp);
  float shed_var = sensors.getTempF(shed_temp);
  //float relay_var = sensors.getTempF(relay_temp);
  //TODO: I still need to solder this wire into the board

<<<<<<< HEAD

  //BMP280 (outside sensor)
  //float ws_temp_var = bme.readTemperature();

  //Variables
//  float dk_avg = (cricket_var + jango_var) / 2;
//  float shed_avg = (tempF2 + shed_var) / 2;
//  float outside_avg = (ws_temp_var + outside_var) / 2;

  /**************************************************************************************************
      Light Loop
  **************************************************************************************************/
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
/*   tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
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
      Serial.print( " LIGHT IS ON  ");
    }
    else {
      digitalWrite(dk_lights, HIGH);
      Serial.print( " LIGHT IS OFF  ");
    }
    light_previousMillis = currentMillis;
  } */
  /****************************************************************************************************
     Fans Loop and Temperature Variables
   ****************************************************************************************************/
/*   if ((currentMillis - fan_previousMillis >= fan_time)) {
    if (outside_avg > 21) {
      if (dk_avg > 60) {
        digitalWrite(dk_fans, HIGH);
        window_var = 2;
        Serial.print("   DK FAN IS ON  ");
      }
    }
    else if (dk_avg <= 60) {
      digitalWrite(dk_fans, LOW);
      window_var = 1;
      Serial.print("   DK FAN IS OFF  ");
    }
    else if (outside_avg <= 21) {
      if (dk_avg >= 65) {
        digitalWrite(dk_fans, HIGH);
        window_var = 1;
        Serial.print("  DK FAN IS ON  ");
      }
      else if (dk_avg < 65) {
        digitalWrite(dk_fans, LOW);
        window_var = 2;
        Serial.print("  DK FAN IS OFF  ");
      }
    }
    //Shed Fan Loop
    if (shed_avg >= 70) {
      digitalWrite (shed_fan, HIGH);
      Serial.print("   Shed FAN IS ON  ");
    }
    else {
      digitalWrite (shed_fan, LOW);
      Serial.print("   Shed FAN IS OFF  ");
    }
    fan_previousMillis = currentMillis;
  } */

  /*******************************************************************************************************
     Computer Fan Loop
   *******************************************************************************************************/
/*   if (currentMillis - comp_previousMillis >= compfan_time) {
    if (tempF1 >= 65) {
      digitalWrite (comp_fans, LOW); //The relay requires a low input to trigger the relay
      Serial.print("   Comp FAN IS ON  ");
    }
    else {
      digitalWrite (comp_fans, HIGH);
      Serial.print("   Comp FAN IS OFF  ");
    }
    comp_previousMillis = currentMillis;
  }
 */
  /*******************************************************************************************************
      IR Heater Loop
  ********************************************************************************************************/
/*   if (currentMillis - heater_previousMillis >= heater_time) {
    if ((outside_avg <= 0) && (dk_avg < 45)) {
      digitalWrite(heater, HIGH);
      Serial.print("   Heater IS ON  ");
      //TODO: i do not need a previous millis becuase i want this to just run the first hour when the system is turned on. Still need to figure this out.
    }
    else {
      digitalWrite(heater, LOW);
      Serial.print("   Heater IS OFF  ");
    }
    heater_previousMillis = currentMillis;
  } */
  /******************************************************************************************
    Loop for Window Motor
  *******************************************************************************************/
  //NEED TO FIGURE OUT THE TEMPERATURES
/*   int  close_value = digitalRead(cl_switch);
  int open_value = digitalRead(op_switch);
  delay(100); */

  /*************************************
     Window going from CLOSE to OPEN
  **************************************/
/*   if ((window_var == 1) && (currentMillis - win_previousMillis >= win_time)) {
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
      Serial.println("   Window IS OPEN  ");
      digitalWrite(win_relay, HIGH);//turning off the 12V power to the motor
      window_var = 0;
    }
  } */

  /************************************
     Window going from OPEN to CLOSE
  *************************************/
/*   if ((window_var == 2) && (currentMillis - win_previousMillis >= win_time)) {
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
      Serial.println("   Window IS CLOSED  ");
      digitalWrite(win_relay, HIGH);//turning off the 12V power to the motor
      window_var = 0;
    }
  } */
  /***********************************************************************************************************
    Serial Print
  ***********************************************************************************************************/
/*   if ((currentMillis - print_previousMillis) >= print_time) {

    DateTime now = rtc.now();
    Serial.print(now.year(), DEC); Serial.print('/'); Serial.print(now.month(), DEC); Serial.print('/');
    Serial.print(now.day(), DEC); Serial.print(") "); Serial.print(now.hour(), DEC); Serial.print(':');
    Serial.print(now.minute(), DEC); Serial.print(':'); Serial.println(now.second(), DEC);

    Serial.print("dk_fans "); Serial.print(digitalRead(dk_fans)); Serial.print(" shed_fan ");Serial.print(digitalRead(shed_fan));

    Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));

    Serial.print(" Humidity = "); Serial.print(bme.readHumidity()); Serial.println(" %"); */

    Serial.print("Shed Temp: "); Serial.print(shed_var); //Serial.print("   Comp Temp: "); Serial.print(tempF1);
=======
    Serial.print("Shed Temp: "); Serial.print(shed_var); Serial.print("   Comp Temp: "); Serial.print(tempF1);
>>>>>>> c8aac9238e4c800658d81d2e926d68da5eeacffd
    Serial.print("  Cricket Temp: "); Serial.print(cricket_var); Serial.print("  Jango Temp: "); Serial.print(jango_var);
    //Serial.print("  Relay Temp:"); Serial.print(relay_var);
    Serial.print("  Outside temp:"); Serial.print(outside_var);
    Serial.print("(*C): "); Serial.print(outsideF_var); Serial.println("(*F): ");

}
