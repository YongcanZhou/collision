

#ifndef DYNAMIC_H_
#define DYNAMIC_H_

static double x, y, z, roll, pitch, yaw;
static std::thread sim_thread_;  
void dynamic(double &x, double &y,double &z,double &roll,double &pitch,double  &yaw);

#endif
