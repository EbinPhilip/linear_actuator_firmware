#include "mode_action.h"
#include "linear_actuator.h"

#include <Arduino.h>

ModeAction::ModeAction(LinearActuator& actuator)
    : actuator_(actuator)
{
}

StartingAction::StartingAction(LinearActuator& actuator)
    : ModeAction(actuator)
{
}

void StartingAction::initialize()
{
    AccelStepper& stepper = actuator_.getStepper();
    stepper.stop();
}

void StartingAction::run()
{
    if (actuator_.getHomingStatus())
    {
        actuator_.changeMode(Mode::HOMING);
    }
}

Mode StartingAction::getMode()
{
    return Mode::STARTING;
}

HomingAction::HomingAction(LinearActuator& actuator)
    : ModeAction(actuator)
{
}

void HomingAction::initialize()
{
    AccelStepper& stepper = actuator_.getStepper();
    stepper.setSpeed(-2400);
}

void HomingAction::run()
{
    
    AccelStepper& stepper = actuator_.getStepper();
    if (actuator_.getHomingStatus())
    {
        if (!actuator_.getEndStopStatus())
        {
            stepper.runSpeed();
        }
        else
        {
            stepper.stop();
            actuator_.changeMode(Mode::ADJUSTING);
        }
    }
    else
    {
        stepper.stop();
        actuator_.changeMode(Mode::STARTING);
    }
}

Mode HomingAction::getMode()
{
    return Mode::HOMING;
}

AdjustingAction::AdjustingAction(LinearActuator& actuator)
    : ModeAction(actuator)
{
}

void AdjustingAction::initialize()
{
    AccelStepper& stepper = actuator_.getStepper();
    stepper.setCurrentPosition(0);
    stepper.moveTo(1200);
    stepper.setSpeed(2400.0);
}

void AdjustingAction::run()
{
    AccelStepper& stepper = actuator_.getStepper();
    if (actuator_.getHomingStatus())
    {
        if(stepper.distanceToGo())
        {
            stepper.runSpeed();
        }
        else
        {
            stepper.stop();
            actuator_.changeMode(Mode::FINE_HOMING);
        }
    }
    else
    {
        stepper.stop();
        actuator_.changeMode(Mode::STARTING);
    }
}

Mode AdjustingAction::getMode()
{
    return Mode::ADJUSTING;
}

FineHomingAction::FineHomingAction(LinearActuator& actuator)
    : ModeAction(actuator)
{
}

void FineHomingAction::initialize()
{
    AccelStepper& stepper = actuator_.getStepper();
    stepper.setSpeed(-400.0);
}

void FineHomingAction::run()
{
    AccelStepper& stepper = actuator_.getStepper();
    if (actuator_.getHomingStatus())
    {
        if (actuator_.getEndStopStatus())
        {
            stepper.stop();
            actuator_.changeMode(Mode::READY);
            stepper.setCurrentPosition(0);
        }
        else
        {
            if(stepper.distanceToGo())
            {
                stepper.runSpeed();
            }
            else
            {
                stepper.moveTo(stepper.currentPosition()-1);
                stepper.setSpeed(-400.0);
            }
        }
    }
    else
    {
        stepper.stop();
        actuator_.changeMode(Mode::STARTING);
    }
}

Mode FineHomingAction::getMode()
{
    return Mode::FINE_HOMING;
}


ReadyAction::ReadyAction(LinearActuator& actuator)
    : ModeAction(actuator)
{
}

void ReadyAction::initialize()
{
    AccelStepper& stepper = actuator_.getStepper();
    stepper.stop();
}

void ReadyAction::run()
{
    if (!actuator_.isEnabled())
    {
        // do nothing
    }
    else
    {
        if (actuator_.getMaxAcceleration()>0.00
        && actuator_.getMaxPosition()>0.00 && actuator_.getMaxSpeed()>0.00)
        {
            actuator_.changeMode(Mode::ACTIVE);
        }
    }
}

Mode ReadyAction::getMode()
{
    return Mode::READY;
}


ActiveAction::ActiveAction(LinearActuator& actuator)
    : ModeAction(actuator)
{
}

void ActiveAction::initialize()
{
}

void ActiveAction::run()
{
    AccelStepper& stepper = actuator_.getStepper();
    if (!actuator_.isEnabled())
    {
        actuator_.changeMode(Mode::READY);
    }

    if (stepper.targetPosition() != actuator_.getTargetPosition())
    {
        stepper.moveTo(actuator_.getTargetPosition());
        stepper.setSpeed(actuator_.getTargetSpeed());
    }

    if (stepper.distanceToGo()!= 0 &&
    !(actuator_.getEndStopStatus() && abs(stepper.currentPosition())>200) )
    {
        stepper.runSpeed();
    }
    else
    {
        stepper.stop();
    }
}

Mode ActiveAction::getMode()
{
    return Mode::ACTIVE;
}