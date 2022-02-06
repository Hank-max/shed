/* Example sketch to control a stepper motor with TB6600 stepper motor driver and Arduino without a library: number of revolutions, speed and direction. More info: https://www.makerguides.com */
// Define stepper motor connections and steps per revolution:
#define dirPin 26
#define stepPin 28
#define win_relay 36
#define op_switch 25
#define cl_switch 29
#define sensor1 3 //this is the sensor inside of the pi case
#define stepsPerRevolution 1600*2
long win_time = 15000;
unsigned long win_previousMillis = 0;

#include "DHT.h"
#define DHTTYPE DHT11 //leaving this here because i may get the DHT22
DHT dht1(sensor1, DHTTYPE);//this is the sensor inside of the pi case

void setup() {
  // Declare pins as output:
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(win_relay, OUTPUT);
  pinMode(op_switch, INPUT);
  pinMode(cl_switch, INPUT);

  //digitalWrite(dirPin, LOW);// this is setting the direction
  dht1.begin();
  Serial.print("   this is working   ");

}
void loop() {
  unsigned long currentMillis = millis();

  // Set the spinning direction clockwise:

  float tempF1 = dht1.readTemperature(true);// this is the sensor inside the pi case
  int  close_value = digitalRead(cl_switch);
  int open_value = digitalRead(op_switch);



  //Serial.print("temp    ");
  //Serial.print(tempF1);
  //Serial.print("    Close ");
  //Serial.print(close_value);
  //Serial.print("    Open ");
  //Serial.println(open_value);
  //
  //delay(1000);
  //&&

  if ((tempF1 > 60) && (currentMillis - win_previousMillis >= win_time)) {
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
}
