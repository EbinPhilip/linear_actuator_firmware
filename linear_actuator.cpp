#include "linear_actuator.h"

#include "Arduino.h"

LinearActuator::LinearActuator(AccelStepper &stepper, volatile bool &end_stop, volatile bool &homing_start)
    : stepper_(stepper), end_stop_(end_stop), homing_start_(homing_start),
      position_cmd_(0), speed_cmd_(0), max_position_(0),
      max_speed_(0), max_acceleration_(0), starting_action_(*this),
      homing_action_(*this), adjusting_action_(*this), fine_homing_action_(*this),
      ready_action_(*this), active_action_(*this), current_action_(&starting_action_)
{
}

AccelStepper& LinearActuator::getStepper()
{
  return stepper_;
}

bool LinearActuator::getEndStopStatus()
{
  return end_stop_;
}

bool LinearActuator::getHomingStatus()
{
  return homing_start_;
}

void LinearActuator::changeMode(Mode mode)
{
  switch(mode)
  {
    case Mode::STARTING:
      current_action_ = &starting_action_;
      break;

    case Mode::HOMING:
      current_action_ = &homing_action_;
      break;

    case Mode::ADJUSTING:
      current_action_ = &adjusting_action_;
      break;

    case Mode::FINE_HOMING:
      current_action_ = &fine_homing_action_;
      break;

    case Mode::READY:
      current_action_ = &ready_action_;
      break;
    
    case Mode::ACTIVE:
      current_action_ = &active_action_;
      break;

    default:
      return;
  }
  // Serial.print("Mode change: ");
  // Serial.println((int)mode);
  current_action_->initialize();
}


Mode LinearActuator::getCurrentMode()
{
  return current_action_->getMode();
}

long LinearActuator::getCurrentPosition()
{
  return stepper_.currentPosition();
}

void LinearActuator::setTargetPosition(long position_cmd)
{
  position_cmd_ = position_cmd;
}

void LinearActuator::setTargetSpeed(long speed_cmd)
{
  speed_cmd_ = speed_cmd;
}

void LinearActuator::setMaxPosition(long max_position)
{
  max_position_ = max_position;
}

void LinearActuator::setMaxSpeed(long max_speed)
{
  max_speed_ = max_speed;
}

void LinearActuator::setMaxAcceleration(long max_acceleration)
{
  max_acceleration_ = max_acceleration;
}

long LinearActuator::getMaxPosition()
{
  return max_position_;
}

long LinearActuator::getMaxSpeed()
{
  return max_speed_;
}

long LinearActuator::getMaxAcceleration()
{
  return max_acceleration_;
}

void LinearActuator::run()
{
  current_action_->run();
  // stepper_.runSpeed();
}
