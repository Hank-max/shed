/* Example sketch to control a stepper motor with TB6600 stepper motor driver and Arduino without a library: number of revolutions, speed and direction. More info: https://www.makerguides.com */
// Define stepper motor connections and steps per revolution:
#define dirPin 26
#define stepPin 28
#define win_relay 36
#define op_switch 29
#define cl_switch 25
#define sensor1 3 //this is the sensor inside of the pi case
#define stepsPerRevolution 1600*2
long win_time = 15000;
unsigned long win_previousMillis = 0;

#include "DHT.h"
#define DHTTYPE DHT11 //leaving this here because i may get the DHT22
DHT dht1(sensor1, DHTTYPE);//this is the sensor inside of the pi case


int op_lastButtonState;   // the previous reading from the input pin
int cl_lastButtonState;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

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
  Serial.print("Close ");Serial.print(close_value); Serial.print("     Open "   );Serial.println(open_value);
  delay(100);

  /*
     Window is going from CLOSE to OPEN
  */
  if ((tempF1 > 60) && (currentMillis - win_previousMillis >= win_time)) {
    op_lastButtonState = LOW;
    if ( open_value != op_lastButtonState) {
      lastDebounceTime = currentMillis;
    }
    if ((open_value == LOW) && (close_value == HIGH) && ((currentMillis - lastDebounceTime) > debounceDelay)) {
      digitalWrite(win_relay, LOW);
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
      digitalWrite(win_relay, HIGH);
    }
  }

  /*
     Window is going from OPEN to CLOSE
  */
  if ((tempF1 < 60) && (currentMillis - win_previousMillis >= win_time)) {
    cl_lastButtonState = LOW;
    if ( close_value != cl_lastButtonState) {
      lastDebounceTime = currentMillis;
    }
    if ((open_value == HIGH) && (close_value == LOW) && ((currentMillis - lastDebounceTime) > debounceDelay)) {
      digitalWrite(win_relay, LOW);
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
      digitalWrite(win_relay, HIGH);
    }
  }
}
