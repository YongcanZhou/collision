
#include <iostream>
#include <thread>
#include <mutex>

#include "dynamic_simulator.h"

using namespace std;
namespace zyc{
        std::thread sim_thread_;  
std::mutex sim_mutex_;
static double x{0.0}, y{0.0}, z{0.0}, roll{0.0}, pitch{0.0}, yaw{0.0};

auto sim_thread_fun()->void{
        while (true){
                // critical section (exclusive access to pose signaled by locking mutex):
                // 当一个线程使用特定互斥量锁住共享数据时，其他线程想要访问锁住的数据，都必须等到之前那个线程堆数据进行解锁后，才能进行访问。
                
                {
                        std::lock_guard<std::mutex> guard(sim_mutex_);//guard为局部变量，分配在栈上，超出作用域即调用析构函数
                        x += 0.05;
                        y += 0.07;
                        z += 0.06;
                        roll += 0.1;
                        pitch += 0.3;
                        yaw += 0.2;
                }
                // per 10ms
                std::this_thread::sleep_for(std::chrono::nanoseconds(10000000));//TODO:sleep_until
        }
};

auto init_simlator()->void{
        if  ( sim_thread_.joinable())
        {
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

