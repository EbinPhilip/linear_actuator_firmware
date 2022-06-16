#ifndef __MODE_ACTION_H__
#define __MODE_ACTION_H__

class LinearActuator;
enum class Mode;

class ModeAction
{
public:
    ModeAction(LinearActuator& actuator);
    virtual void initialize() = 0;
    virtual void run() = 0;
    virtual Mode getMode() = 0;
protected:
    LinearActuator& actuator_;
};

class StartingAction : public ModeAction
{
public:
    StartingAction(LinearActuator& actuator);
    virtual void initialize() override;
    virtual void run() override;
    virtual Mode getMode() override;
};

class HomingAction : public ModeAction
{
public:
    HomingAction(LinearActuator& actuator);
    virtual void initialize() override;
    virtual void run() override;
    virtual Mode getMode() override;
};

class AdjustingAction : public ModeAction
{
public:
    AdjustingAction(LinearActuator& actuator);
    virtual void initialize() override;
    virtual void run() override;
    virtual Mode getMode() override;
};

class FineHomingAction : public ModeAction
{
public:
    FineHomingAction(LinearActuator& actuator);
    virtual void initialize() override;
    virtual void run() override;
    virtual Mode getMode() override;
};

class ReadyAction : public ModeAction
{
public:
    ReadyAction(LinearActuator& actuator);
    virtual void initialize() override;
    virtual void run() override;
    virtual Mode getMode() override;
};

class ActiveAction : public ModeAction
{
public:
    ActiveAction(LinearActuator& actuator);
    virtual void initialize() override;
    virtual void run() override;
    virtual Mode getMode() override;
};

class ErrorAction : public ModeAction
{
public:
    ErrorAction(LinearActuator& actuator);
    virtual void initialize() override;
    virtual void run() override;
    virtual Mode getMode() override;
};

#endif