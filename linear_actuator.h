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
    float getCurrentSpeed();

    void setTargetPosition(long position_cmd);
    void setTargetSpeed(float speed_cmd);

    long getTargetPosition();
    float getTargetSpeed();

    void setMaxPosition(long max_position);
    void setMaxSpeed(float max_speed);
    void setMaxAcceleration(float max_acceleration);

    bool isEnabled();
    void setEnabled(bool enabled);

    long getMaxPosition();
    float getMaxSpeed();
    float getMaxAcceleration();

    void run();

protected:
    AccelStepper& stepper_;
    volatile bool& end_stop_;
    volatile bool& homing_start_;

    long position_cmd_;
    float speed_cmd_;

    long max_position_;
    float max_speed_;
    float max_acceleration_;

    bool enabled_;

    StartingAction starting_action_;
    HomingAction homing_action_;
    AdjustingAction adjusting_action_;
    FineHomingAction fine_homing_action_;
    ReadyAction ready_action_;
    ActiveAction active_action_;

    ModeAction* current_action_;
};

#endif