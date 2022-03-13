#ifndef __LINEAR_ACTUATOR_H__
#define __LINEAR_ACTUATOR_H__

#include <AccelStepper.h>

#include "mode_action.h"

enum class Mode
{
    STARTING,
    HOMING,
    ADJUSTING,
    FINE_HOMING,
    READY,
    ACTIVE
};

class LinearActuator
{
public:

    LinearActuator(AccelStepper& stepper, volatile bool& end_stop, volatile bool& homing_start);

    AccelStepper& getStepper();
    bool getEndStopStatus();
    bool getHomingStatus();

    void changeMode(Mode mode);
    Mode getCurrentMode();

    long getCurrentPosition();
    long getCurrentSpeed();

    void setTargetPosition(long position_cmd);
    void setTargetSpeed(long speed_cmd);

    void setMaxPosition(long max_position);
    void setMaxSpeed(long max_speed);
    void setMaxAcceleration(long max_acceleration);

    long getMaxPosition();
    long getMaxSpeed();
    long getMaxAcceleration();

    void run();

protected:
    AccelStepper& stepper_;
    volatile bool& end_stop_;
    volatile bool& homing_start_;

    long position_cmd_;
    long speed_cmd_;

    long max_position_;
    long max_speed_;
    long max_acceleration_;

    StartingAction starting_action_;
    HomingAction homing_action_;
    AdjustingAction adjusting_action_;
    FineHomingAction fine_homing_action_;
    ReadyAction ready_action_;
    ActiveAction active_action_;

    ModeAction* current_action_;
};

#endif