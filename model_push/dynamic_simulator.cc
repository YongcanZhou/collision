
#include <iostream>
#include <thread>
#include <mutex>

#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <Eigen/Dense>

#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"

#include "dynamic_simulator.h"

using namespace std;
using namespace fcl;
using std::map;
using std::pair;
using std::string;
using std::vector;
using fcl::AngleAxis;
using fcl::Transform3;
using fcl::Vector3;

struct BoxSpecification {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    fcl::Vector3<double> size;
    fcl::Transform3<double> X_FB;
  };

namespace zyc
{
  
  std::thread sim_thread_;  
  std::mutex sim_mutex_;

  // Simple specification for defining a box collision object. 
  // Specifies the dimensions and pose of the box in some frame F (X_FB).
  // X_FB is the "pose" of frame B in frame F.

  static double x{0.0}, y{0.0}, z{10.0}, roll{0.0}, pitch{0.0}, yaw{0.0}, 
                duration{0.0}, delta_t{0.001}, g{9.8}, delta_z{0.0}, v_0{0.0};

  auto sim_thread_fun()->void{
    //box1
    const double size_1 = 1;
    BoxSpecification box_1{fcl::Vector3<double>{size_1, size_1, size_1},
                              fcl::Transform3<double>::Identity()};
    //box2
    const double size_2 = 1;
    BoxSpecification box_2{fcl::Vector3<double>{size_2, size_2, size_2},
                              fcl::Transform3<double>::Identity()};
    const fcl::Vector3<double> pose_box_2{x, y, z};
    box_2.X_FB.translation() = pose_box_2;

    // using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<double>>;
    // CollisionGeometryPtr_t box_geometry_A(new fcl::Box<double>(box_1.size));
    // CollisionGeometryPtr_t box_geometry_B(new fcl::Box<double>(box_2.size));
    // fcl::CollisionObject<double> box_A(box_geometry_A, box_1.X_FB);
    // fcl::CollisionObject<double> box_B(box_geometry_B, box_2.X_FB);

    // // Compute collision - single contact and enable contact.
    // fcl::CollisionRequest<double> Request;
    // // Request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    // fcl::CollisionResult<double> Result;
    
    // // std::vector<fcl::Contact<double>> contacts;
    // // Result.getContacts(contacts);

    while (true){
      // TODO: fcl
      box_2.X_FB.translation() << x, y, z;
      cout <<"pose" << box_2.X_FB.translation() << endl;
      // int num_contacts = fcl::collide(&box_A, &box_B, Request, Result);
      // cout<<"contact"<<num_contacts<<endl;
      // if (num_contacts!=0){
      //   cout<<"contact"<<num_contacts<<endl;
      // }
      
      // start time
      auto start = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> guard(sim_mutex_);//guard为局部变量，分配在栈上，超出作用域即调用析构函数
        x += 0;
        y += 0;
        duration += delta_t;
        v_0 = g*duration;
        delta_z = v_0*delta_t+1/2*g*pow(delta_t,2);
        z -= 0.01*(delta_z);//slow 100X
        roll += 0;
        pitch += 0;
        yaw += 0;
      }
      // per 1ms
      // std::this_thread::sleep_for(std::chrono::nanoseconds(10000000));
      std::this_thread::sleep_until(start + std::chrono::nanoseconds(1000000));
      //verify time 
      // auto duration = chrono::duration_cast<chrono::milliseconds>(std::chrono::steady_clock::now() - start);
      // cout << "Operation duration : " << duration.count() << "ms" << endl;
    }
  };

  auto init_simulator()->void{
    if  ( sim_thread_.joinable()){
      return;
    }
    else{
      sim_thread_ = std::thread(sim_thread_fun); 
    }
  }

  void DynamicSimulator(double &x_, double &y_,double &z_,double &roll_,double &pitch_,double  &yaw_){    
    std::lock_guard<std::mutex> guard(sim_mutex_);//guard为局部变量，分配在栈上，超出作用域即调用析构函数
    x_ = x;
    y_ = y;
    z_ = z;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;    
  }
}

