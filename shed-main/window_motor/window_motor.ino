/* Example sketch to control a stepper motor with TB6600 stepper motor driver and Arduino
  without a library: number of revolutions, speed and direction. More info:
  https://www.makerguides.com */

// Define stepper motor connections and steps per revolution:
#define dirPin 26
#define stepPin 28
#define win_relay 36 // this is going the relay to the 12V power supply

#define stepsPerRevolution 1600
int win_relayState;



void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(win_relay, OUTPUT);
  // Set the spinning direction CW/CCW:
  digitalWrite(dirPin, LOW);
  digitalWrite(win_relay, HIGH);
}
void loop() {
  // These four lines result in 1 step:
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(500);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(500);
}
