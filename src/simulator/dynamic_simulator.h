//
// Created by ZHOUYC on 2022/6/14.
//

#ifndef DYNAMIC_H_
#define DYNAMIC_H_

using namespace Eigen;
namespace zyc{
  auto InitSimulator()->void;
  auto SimThreadFun()->void;
  auto DynamicSimulator(std::array<double,7*16> &link_pm_)->void;
  auto col_thread()->void;
}

#endif
