
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
using std::map;
using std::pair;
using std::string;
using std::vector;

namespace zyc
{
  std::thread sim_thread_;  
  std::mutex sim_mutex_;

  static double x{0.0}, y{0.0}, z{10.0}, roll{0.0}, pitch{0.0}, yaw{0.0}, 
                duration{0.0}, delta_t{0.001}, g{9.8}, delta_z{0.0}, v_0{0.0};
  auto start_first = std::chrono::steady_clock::now();

  auto sim_thread_fun()->void{
    while (true){
      // TODO: fcl

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

  void DynamicSimulator(double &x_, double &y_,double &z_,double &roll_,double &pitch_,double  &yaw_)
  {    
    std::lock_guard<std::mutex> guard(sim_mutex_);//guard为局部变量，分配在栈上，超出作用域即调用析构函数
    x_ = x;
    y_ = y;
    z_ = z;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;    
  }
}

