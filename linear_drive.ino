#include <AccelStepper.h>
#include <ros.h>
#include <linear_actuator_controller/LinearActuatorFeedback.h>
#include <linear_actuator_controller/LinearActuatorInput.h>

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

ros::NodeHandle nh;
linear_actuator_controller::LinearActuatorFeedback feedback;
ros::Publisher chatter("feedback", &feedback);

void inputCb( const linear_actuator_controller::LinearActuatorInput& input)
{
  actuator.setTargetPosition(input.position);
  actuator.setTargetSpeed(input.speed);
  actuator.setMaxAcceleration(input.acceleration_max);
  actuator.setMaxPosition(input.position_max);
  actuator.setMaxSpeed(input.speed_max);
  actuator.setEnabled(input.enabled);

  feedback.position = actuator.getCurrentPosition();
  feedback.speed = actuator.getCurrentSpeed();
  feedback.active = (actuator.getCurrentMode() == Mode::ACTIVE);
  chatter.publish( &feedback );
}

ros::Subscriber<linear_actuator_controller::LinearActuatorInput> sub("input", &inputCb );

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

  // case where actuator is at 0 position when powered on
  // interrupt that responds to pin level change will miss this case
  if (digitalRead(end_stop_pin) == LOW)
  {
    end_stop = true;
  }

  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}


void loop() {
  Mode mode = actuator.getCurrentMode();
  if (mode == Mode::READY || mode == Mode::ACTIVE)
  {
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }
  actuator.run();
  nh.spinOnce();
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
