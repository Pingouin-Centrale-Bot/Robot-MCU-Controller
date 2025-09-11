#include "ProgramRemoteControl.h"

ProgramRemoteControl::ProgramRemoteControl(Motion *m, Lift *g, Lift *d, IHM *i, Lidar *l)
    : motion(m), liftG(g), liftD(d), ihm(i), lidar(l)
{
    rc = new RemoteControl(motion, liftG, liftD);
}

void ProgramRemoteControl::init()
{
}

void ProgramRemoteControl::run()
{
    rc->start();
    // Should never return.
}
