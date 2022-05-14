
#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"

#ifndef DYNAMIC_H_
#define DYNAMIC_H_

namespace zyc{
  void DynamicSimulator(double &x, double &y,double &z,double &roll,double &pitch,double  &yaw);
  auto InitSimulator()->void;
  auto SimThreadFun()->void;  
  auto MoveUp()->void;
  auto MoveDown()->void;
  auto Move()->void;
}

#endif
