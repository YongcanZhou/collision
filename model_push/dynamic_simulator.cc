
#include <iostream>
#include <thread>
#include <mutex>

#include "dynamic_simulator.h"

using namespace std;
namespace zyc
{
  std::thread sim_thread_;  
  std::mutex sim_mutex_;
  static double x{0.0}, y{0.0}, z{0.0}, roll{0.0}, pitch{0.0}, yaw{0.0};

  auto sim_thread_fun()->void{
    while (true){
      // TODO: fcl

      // 记录开始时间
      auto start = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> guard(sim_mutex_);//guard为局部变量，分配在栈上，超出作用域即调用析构函数
        x += 0.0005;
        y += 0.0007;
        z += 0.0006;
        roll += 0.001;
        pitch += 0.003;
        yaw += 0.002;
      }
      // per 10ms
      // std::this_thread::sleep_for(std::chrono::nanoseconds(10000000));
      std::this_thread::sleep_until(start + std::chrono::nanoseconds(10000000));
      // std::chrono::duration<double, std::milli> elapsed {std::chrono::steady_clock::now() - start};
      // std::cout << "Waited " << elapsed.count() << " ms\n";
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

