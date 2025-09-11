#include "ProgramRemoteControl.h"

ProgramRemoteControl::ProgramRemoteControl(Motion *m, Lift *g, Lift *d, IHM *i, Lidar *l)
    : motion(m), liftG(g), liftD(d), ihm(i), lidar(l)
{
    battery = new Battery(m, g, d);
    rc = new RemoteControl(motion, liftG, liftD, battery);
}

void ProgramRemoteControl::init()
{
    battery->start_monitoring(ihm);
}

void ProgramRemoteControl::run()
{
    rc->start();
    // Should never return.
}
