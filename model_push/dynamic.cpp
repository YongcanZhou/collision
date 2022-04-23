
#include <iostream>
#include <thread>

#include "dynamic.h"

using namespace std;

void dynamic(double &x, double &y,double &z,double &roll,double &pitch,double  &yaw){
        // record start time
        auto start = std::chrono::steady_clock::now();

        // std::cout <<"x "<<x<<std::endl;

        if  ( sim_thread_.joinable())
        {
                // std::cout <<"x "<<x<<std::endl;
                // double *xp1 = &x;
                // printf("%p\n",&x);
                return;
        }
        else{
        //using lambda
        sim_thread_ = std::thread([&]()
        {
                // std::chrono::steady_clock
                // 记录开始时间
                auto start = std::chrono::steady_clock::now();

                while (true){
                x += 0.05;
                y += 0.07;
                z += 0.06;
                roll += 0.1;
                pitch += 0.3;
                yaw += 0.2;//mutex  protect

                // per 1s
                std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000));
                }
        }); 
        }
}
