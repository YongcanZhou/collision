
#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"

#ifndef DYNAMIC_H_
#define DYNAMIC_H_

namespace zyc{
  void DynamicSimulator(double &x, double &y,double &z,double &roll,double &pitch,double  &yaw);
  auto init_simulator()->void;
  template <typename S>
  void collision_box_box(fcl::GJKSolverType solver_type, S test_tolerance);
  struct BoxSpecification {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    fcl::Vector3<double> size;
    fcl::Transform3<double> X_FB;
  };

}

#endif
