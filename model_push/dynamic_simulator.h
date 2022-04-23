

#ifndef DYNAMIC_H_
#define DYNAMIC_H_

static double x, y, z, roll, pitch, yaw;
static std::thread sim_thread_;  
static std::mutex sim_mutex_;
void DynamicSimulator(double &x, double &y,double &z,double &roll,double &pitch,double  &yaw);

#endif
