
//
// Created by ZHOUYC on 2022/6/14.
//

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

  static array<double ,7*16> link_pm;
  static double link_inv[16],output_inv[6]{393.0,0.0,642.0,0.0,1.5708,0.0},delta_axis01{0.0};

  auto SimThreadFun()->void{

    Matrix4d Matrix4d_pm_new=Matrix4d::Zero();
    Matrix4d Matrix4d_pm;
    Matrix4d trsf,trsf_test,trsf_inv,temp;

    PumaParam param;
    param.a1 = 40;
    param.a2 = 275;
    param.a3 = 25;
    param.d1 = 342;
    param.d3 = 0.0;
    param.d4 = 280;
    param.tool0_pe[2] = 73;
    auto m = aris::dynamic::createModelPuma(param);
    dynamic_cast<aris::dynamic::GeneralMotion &>(m->generalMotionPool().at(0)).setPoseType(
            GeneralMotion::PoseType::EULER123);

    //inverse kinematic
    output_inv[0] += 1;
//    output_inv[2] -= 5;
    m->setOutputPos(output_inv);
    if (m->inverseKinematics()) {
      std::cout << "inverse kinematic failed" << std::endl;
    }//0成功 1失败
    for(int i=0;i<7;++i){
      m->partPool().at(i).getPm(link_inv);
      for(int j=0;j<16;++j){
        link_pm[j+i*16] = link_inv[j];
      }
    }

    //转换末端位姿
    m->partPool().at(6).getPm(link_inv);
    //一维数组改为二维
    int a=0;
    int b=0;
    for (int m=0;m<16;m++)
    {
      a=m/4;
      b=m%4;
      Matrix4d_pm(a,b)=link_inv[m];
    }
//     Matrix4d_pm = trsf.inverse() * Matrix4d_pm;
    for (auto q = 0; q < 4; q++)
    {
      for (auto w = 0; w < 4; w++)
      {
        Matrix4d_pm_new(q,w)=0;
        for (auto k = 0; k < 4; k++)
        {
          Matrix4d_pm_new(q,w) += Matrix4d_pm(q,k) *trsf_inv(k,w) ;
        }
      }
    }
    //二维变一维 获取
    for (int c = 0; c < 4; ++c){
      for (int d = 0; d < 4; ++d){
        link_pm[96+c*4+d] =  Matrix4d_pm_new(c,d);
      }
    }
  }
  auto col_thread()->void{

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

