#pragma once
#include "ProgramBase.h"
#include "modules/Motion.h"
#include "modules/Lift.h"
#include "modules/IHM.h"
#include "modules/Lidar.h"
#include "modules/Battery.h"
#include "modules/RemoteControl.h"

class ProgramRemoteControl : public ProgramBase {
public:
    ProgramRemoteControl(Motion* m, Lift* g, Lift* d, IHM* i, Lidar* l);

    void init() override;
    void run() override;

private:
    Motion* motion;
    Lift* liftG;
    Lift* liftD;
    IHM* ihm;
    Lidar* lidar;
    Battery* battery;
    RemoteControl *rc;
};
