#pragma once
#include "ProgramBase.h"
#include "modules/Motion.h"
#include "modules/Lift.h"
#include "modules/IHM.h"
#include "modules/Lidar.h"

class Program3 : public ProgramBase {
public:
    Program3(Motion* m, Lift* g, Lift* d, IHM* i, Lidar* l);

    void init() override;
    void run() override;

private:
    Motion* motion;
    Lift* liftG;
    Lift* liftD;
    IHM* ihm;
    Lidar* lidar;
};
