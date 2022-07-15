
#include <iostream>
#include <thread>
#include <mutex>
#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <aris.hpp>
#include <iostream>
#include <array>


#include "dynamic_simulator.h"

using namespace std;
using std::map;
using std::pair;
using std::string;
using std::vector;
using namespace aris::dynamic;
using namespace Eigen;

namespace zyc
{
  std::thread sim_thread_;  
  std::mutex  sim_mutex_;

  static double x{0.0}, y{0.0}, z{5.0}, roll{0.0}, pitch{0.0}, yaw{0.0}, 
                duration{0.0}, delta_t{0.001}, g{-9.8}, delta_z{0.0}, velocity{0.0},
                link_position[6*16],delta_axis01{0.0};
  static array<double ,7*16> link_pm;

  static double link_inv[16],link_test[16],output_inv[6]{393.0,0.0,642.0,0.0,1.5708,0.0},
                pm[4][4],ee_inv[16];

  static Eigen::Matrix4d foo1 = (Eigen::Matrix4d() << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16).finished();


  static Eigen::Matrix4d test = [] {
        Eigen::Matrix4d tmp;
        tmp << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        return tmp;
    }();

    auto SimThreadFun()->void{
      PumaParam param;

      param.a1 = 40;
      param.a2 = 275;
      param.a3 = 25;
      param.d1 = 342;
      param.d3 = 0.0;
      param.d4 = 280;
      param.tool0_pe[2] = 73;

      Matrix4d Matrix4d_pm_new=Matrix4d::Zero();
      Matrix4d Matrix4d_pm_tr=Matrix4d::Zero();
      Matrix4d temp_1=Matrix4d::Identity();
      Matrix4d Matrix4d_pm;

      Matrix4d trsf,trsf_test,trsf_inv,temp;

      trsf<< 0.0  , 0.0  , -1.0 , 642.0,
              0.0  , 1.0  , 0.0 , 0.0,
              1.0 , 0.0  , 0.0 ,-393.0,
              0.0  , 0.0  , 0.0 , 1.0;
      trsf_test<< 0.0  , 0.0  , 1.0 , 393.0,
                  0.0  , 1.0  , 0.0 , 0.0,
                  -1.0 , 0.0  , 0.0 ,642.0,
                  0.0  , 0.0  , 0.0 , 1.0;
      trsf_inv = trsf_test.inverse();
      auto m = aris::dynamic::createModelPuma(param);
      dynamic_cast<aris::dynamic::GeneralMotion &>(m->generalMotionPool().at(0)).setPoseType(
              GeneralMotion::PoseType::POSE_MATRIX);

      double input[6]{delta_axis01, 0.0, 0.0, 0.0, 0.0, 0.0}, link[16];
      delta_axis01 +=1.5;

      //forward kinematic
      m->setInputPos(input);
      m->forwardKinematics();
//      m->getOutputPos()link;
      for(int i=0;i<7;++i){
        m->partPool().at(i).getPm(link);
        for(int j=0;j<16;++j){
          link_pm[j+i*16] = link[j];
//              if(i==6){std::cout<<"num"<<j+i*16<<":"<<link[j]<<" ";}
//            std::cout<<"link"<<j+i*16<<":"<<link[j]<<" ";
        }
      }

//      std::cout<<"-------------------------"<<std::endl;
      m->partPool().at(6).getPm(link);
      //一维数组改为二维
      int a=0;
      int b=0;
      for (int m=0;m<16;m++)
      {
        a=m/4;
        b=m%4;
        Matrix4d_pm(a,b)=link[m];
        temp (a,b)= link_pm[96+m];
//        std::cout<<"ee"<<a<<" "<<b<<" "<<Matrix4d_pm(a,b)<<setw( 16 );
      }
//      std::cout<<"-------------------------"<<std::endl;
//     Matrix4d_pm = trsf.inverse() * Matrix4d_pm;
      for (auto q = 0; q < 4; q++)
      {
        for (auto w = 0; w < 4; w++)
        {
//          std::cout<<"ee_"<<q<<" "<<w<<" "<<test(q,w)<<setw( 16 );
          Matrix4d_pm_new(q,w)=0;
          for (auto k = 0; k < 4; k++)
          {
            Matrix4d_pm_new(q,w) += Matrix4d_pm(q,k) *trsf_inv(k,w) ;
          }
//          std::cout<<"ee_"<<q<<" "<<w<<" "<<test(q,w)<<setw( 16 );
        }
      }
      //二维变一维 获取
      for (int c = 0; c < 4; ++c){
        for (int d = 0; d < 4; ++d){
          link_pm[96+c*4+d] =  Matrix4d_pm_new(c,d);
//        cout << "ee_new " <<96+c*4+d<< Matrix4d_pm_new(c,d)<<std::endl ;
        }
      }
      Matrix4d Matrix4d_pm_new=Matrix4d::Zero();
      Matrix4d Matrix4d_pm_tr=Matrix4d::Zero();
      Matrix4d temp_1=Matrix4d::Identity();
      Matrix4d Matrix4d_pm;

      Matrix4d trsf,trsf_test,trsf_inv,temp;
/*
      trsf_test<< 0.0  , 0.0  , 1.0 , 393.0,
              0.0  , 1.0  , 0.0 , 0.0,
              -1.0 , 0.0  , 0.0 ,642.0,
              0.0  , 0.0  , 0.0 , 1.0;
      trsf_inv = trsf_test.inverse();

      auto m = aris::dynamic::createModelPuma(param);
      dynamic_cast<aris::dynamic::GeneralMotion &>(m->generalMotionPool().at(0)).setPoseType(
              GeneralMotion::PoseType::EULER123);

      //inverse kinematic
      output_inv[0] += 5;
      output_inv[2] -= 5;
      m->setOutputPos(output_inv);
      if (m->inverseKinematics()) {
        std::cout << "inverse kinematic failed" << std::endl;
      }//0成功 1失败

      for(int i=0;i<7;++i){
        m->partPool().at(i).getPm(link_inv);
        for(int j=0;j<16;++j){
          link_pm[j+i*16] = link_inv[j];
//              if(i==6){std::cout<<"num"<<j+i*16<<":"<<link[j]<<" ";}
//            std::cout<<"link"<<j+i*16<<":"<<link[j]<<" ";
        }
      }

//      std::cout<<"-------------------------"<<std::endl;
      m->partPool().at(6).getPm(link_inv);
      //一维数组改为二维
      int a=0;
      int b=0;
      for (int m=0;m<16;m++)
      {
        a=m/4;
        b=m%4;
        Matrix4d_pm(a,b)=link_inv[m];
//        std::cout<<"ee"<<a<<" "<<b<<" "<<Matrix4d_pm(a,b)<<setw( 16 );
      }
//      std::cout<<"-------------------------"<<std::endl;
//     Matrix4d_pm = trsf.inverse() * Matrix4d_pm;
      for (auto q = 0; q < 4; q++)
      {
        for (auto w = 0; w < 4; w++)
        {
//          std::cout<<"ee_"<<q<<" "<<w<<" "<<test(q,w)<<setw( 16 );
          Matrix4d_pm_new(q,w)=0;
          for (auto k = 0; k < 4; k++)
          {
            Matrix4d_pm_new(q,w) += Matrix4d_pm(q,k) *trsf_inv(k,w) ;
          }
//          std::cout<<"ee_"<<q<<" "<<w<<" "<<test(q,w)<<setw( 16 );
        }
      }
      //二维变一维 获取
      for (int c = 0; c < 4; ++c){
        for (int d = 0; d < 4; ++d){
          link_pm[96+c*4+d] =  Matrix4d_pm_new(c,d);
//        cout << "ee_new " <<96+c*4+d<< Matrix4d_pm_new(c,d)<<std::endl ;
        }
      }*/

//      while (true){
//          auto start = std::chrono::steady_clock::now();
//          {
//              std::lock_guard<std::mutex> guard(sim_mutex_);//guard为局部变量，分配在栈上，超出作用域即调用析构函数
//              x += 0;
//              y += 0;
//              velocity += g*delta_t;
//              delta_z = velocity*delta_t+1/2*g*pow(delta_t,2);
//              z += 0.01*(delta_z);//slow 100X
//              roll += 0;
//              pitch += 0;
//              yaw += 0;
//          }
//          // per 1ms
//          // std::this_thread::sleep_for(std::chrono::nanoseconds(10000000));
//          std::this_thread::sleep_until(start + std::chrono::nanoseconds(1000000));
//          //verify time
//          // auto duration = chrono::duration_cast<chrono::milliseconds>(std::chrono::steady_clock::now() - start);
//          // cout << "Operation duration : " << duration.count() << "ms" << endl;
//      }
  }

    auto InitSimulator()->void {
        if (sim_thread_.joinable()) {
            return;
        } else {
            sim_thread_ = std::thread(SimThreadFun);
        }
    }

  void DynamicSimulator(std::array<double,7*16> &link_pm_){
    std::lock_guard<std::mutex> guard(sim_mutex_);//guard为局部变量，分配在栈上，超出作用域即调用析构函数
    zyc::SimThreadFun();
    link_pm_ = link_pm;
  }
}

