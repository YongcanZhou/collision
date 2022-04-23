
#include <iostream>
#include <thread>
#include <mutex>

#include "dynamic_simulator.h"

using namespace std;

void DynamicSimulator(double &x, double &y,double &z,double &roll,double &pitch,double  &yaw){
        
        std::lock_guard<std::mutex> guard(sim_mutex_);//guard为局部变量，分配在栈上，超出作用域即调用析构函数

        // record start time
        auto start = std::chrono::steady_clock::now();

        if  ( sim_thread_.joinable())
        {
                return;
        }
        else{
        //using lambda
        sim_thread_ = std::thread([&]()
        {
                // std::chrono::steady_clock
                // record start time
                auto start = std::chrono::steady_clock::now();

                while (true){

                // critical section (exclusive access to pose signaled by locking mutex):
                // 当一个线程使用特定互斥量锁住共享数据时，其他线程想要访问锁住的数据，都必须等到之前那个线程堆数据进行解锁后，才能进行访问。
                // sim_mutex_.lock();
                x += 0.05;
                y += 0.07;
                z += 0.06;
                roll += 0.1;
                pitch += 0.3;
                yaw += 0.2;
                // sim_mutex_.unlock();//

                // per 1s
                std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000));
                }
        }); 
        }
}
