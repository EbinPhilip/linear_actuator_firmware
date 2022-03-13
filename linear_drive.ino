#include <AccelStepper.h>

#include "linear_actuator.h"

#define dirPin 8
#define stepPin 9
#define motorInterfaceType 1

volatile bool end_stop = false;
unsigned int end_stop_pin = 3;
unsigned int end_stop_pwr = 7;

volatile bool homing_start = false;
unsigned int homing_start_pin = 2;
unsigned int homing_start_pwr = 6;

unsigned int ledPin = 13;

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
LinearActuator actuator(stepper, end_stop, homing_start);

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(end_stop_pin, INPUT_PULLUP);
  pinMode(end_stop_pwr, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(end_stop_pin), endStopISR, CHANGE);

  pinMode(homing_start_pin, INPUT_PULLUP);
  pinMode(homing_start_pwr, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(homing_start_pin), homingStartISR, CHANGE);
  
  digitalWrite(ledPin, LOW);
  digitalWrite(end_stop_pwr, LOW);
  digitalWrite(homing_start_pwr, LOW);

  stepper.setMaxSpeed(5600);
  stepper.setAcceleration(2400);

  Serial.begin(115200);
}


void loop() {
  actuator.run();  
}

void endStopISR()
{
  if (digitalRead(end_stop_pin) == LOW)
    end_stop = true;
  else
    end_stop = false;
}

void homingStartISR()
{
  if (digitalRead(homing_start_pin) == LOW)
    homing_start = true;
  else
    homing_start = false;
}
