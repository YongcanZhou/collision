#include "threads.h"
#include "occview.h"
#include "ui_mainwindow.h"
#include <sire.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <sire/physics/contact/test/arm_push.hpp>

threadSimulation::threadSimulation(QThread *parent)
    : QThread(parent) {
  running = false;// set to stop detached thread(thread_visual)
}

void threadSimulation::run() {
  XB4Push();
  //BallRebound();
  //XB4Polish();
}

auto threadSimulation::XB4Push() -> void {
  num_contacts = 0;
  //开启仿真轨迹规划
  //auto &sim = sire::simulator::Simulator::instance("H:/contact/collision/sire.xml");
  auto &sim = sire::physics::contact::SimulatorPush::instance("H:/contact/collision/sire.xml");
  //sire::physics::collision::Collision &collision =
  //        sire::physics::collision::Collision::instance("H:/contact/collision/model/STL");
  auto &collision =
           sire::physics::collision::Collision::instance("H:/contact/collision/model/STL");

  //0.393, 0, 0.642, 0, 1.5708, 0
  std::vector<std::array<double, 6>> track_points{{0.570, 0, 0.642, 0, 1.5708, 0}};
  sim.SimPlan(track_points);

  //线程thread_visual，每100ms从sim获取link位姿
  static std::thread thread_visual([&] {
    running = true;
    const double delta_t = 0.001;
    while (!num_contacts && running && !threadStop) {
      auto start = std::chrono::steady_clock::now();
      sim.SetCollisionNum(num_contacts);
      //从sire获取link位姿
      sim.GetLinkPQ(link_pq);
      //通过mainwindow.cpp中槽函数传递link_pm给occview.cpp中的setLinkPm()
      emit updateLinkPQ(link_pq);
      //计算是否碰撞
      collision.CalCollision(link_pq, num_contacts);
      std::this_thread::sleep_until(start + std::chrono::duration<double, std::milli>(delta_t * 1000));
    }
  });
  //分离线程thread_visual
  if (thread_visual.joinable()) {
    thread_visual.detach();
  }
}

auto threadSimulation::BallRebound() -> void {
  num_contacts = 0;
  //开启仿真轨迹规划
  //auto &sim = sire::Simulator::instance("H:/contact/collision/collision.xml");
  //  sire::collision::Collision &collision = sire::collision::Collision::instance("C:/Users/ZHOUYC/Desktop/model_sw/Robot_STL");

  auto &sim = sire::physics::contact::BallRebound::instance();

  //线程thread_visual，每100ms从sim获取link位姿
  static std::thread thread_visual([&] {
    running = true;
    const double delta_t = 0.01;
    while (/*num_contacts == 0 &&*/ running && !threadStop) {
      auto start = std::chrono::steady_clock::now();
      //从aris_sim获取link位姿
      sim.CalContact(sphere_pq, num_contacts);
      //std::cout << "position:"
      //          << "x " << sphere_pq[0] << " y " << sphere_pq[1] << " z " << sphere_pq[2] << std::endl;
      //sim.GetLinkPQ(link_pq);
      //通过mainwindow.cpp中槽函数传递link_pm给occview.cpp中的setLinkPm()
      //emit updateLinkPQ(link_pq);
      emit updateSpherePQ(sphere_pq);
      //计算是否碰撞
      //      collision.CalCollision(link_pq, num_contacts);
      std::this_thread::sleep_until(start + std::chrono::duration<double, std::milli>(delta_t * 1000));
    }
  });
  //分离线程thread_visual
  if (thread_visual.joinable()) {
    thread_visual.detach();
  }
}

auto threadSimulation::XB4Polish() -> void {
  num_contacts = 0;
  //开启仿真轨迹规划
  auto &sim = sire::simulator::Simulator::instance("H:/contact/collision/sire.xml");
  sire::physics::collision::Collision &collision =
          sire::physics::collision::Collision::instance("H:/contact/collision/model/STL");

  std::array<double, 6> track_point;
  if (OccView::finish_norm) {
    //auto track_points = OccView::GetTrackPoints();
    auto CutOver_CAM_pointVecVecs = OccView::GetTrackPoints();
    //track_points 点坐标加法向量 转换为 点坐标加欧拉角
    for (auto i: CutOver_CAM_pointVecVecs) {
      for (auto j: i) {
        Eigen::Matrix3d m;
        Eigen::Vector3d zaxis(0, 0, 1);
        Eigen::Vector3d tmpvec(-j.v1.X(), -j.v1.Y(), -j.v1.Z());
        tmpvec.normalize();
        // 叉乘得到转轴，而且是从z_axis转到normal
        Eigen::Vector3d rot_axis = zaxis.cross(tmpvec);
        rot_axis.normalize();// 归一化
        std::cout << "axis: " << rot_axis.transpose() << std::endl;
        // 点乘得到转角
        double rot_angle = acos(zaxis.dot(tmpvec));
        std::cout << "angle: " << rot_angle * 180.0 / M_PI << std::endl;
        // 得到旋转矩阵
        Eigen::Matrix3d m_rot = Eigen::Matrix3d(Eigen::AngleAxisd(rot_angle, rot_axis));

        // aris默认313，得到欧拉角
        Eigen::Vector3d v_RPY = m_rot.eulerAngles(3, 1, 3);

        track_point[0] = j.p1.X() * 0.001;
        track_point[1] = j.p1.Y() * 0.001;
        track_point[2] = j.p1.Z() * 0.001;
        track_point[3] = v_RPY[0];
        track_point[4] = v_RPY[1];
        track_point[5] = v_RPY[2];
        track_points.push_back(track_point);
      }
    }
    //sim.SimPlan(track_points);
  } else {
    std::cout << "move without normal vertor" << std::endl;
    //0.393, 0, 0.642, 0, 1.5708, 0
    std::vector<std::array<double, 6>> track_points{{0.580, 0, 0.642, 0, 1.5708, 0}};
    sim.SimPlan(track_points);
  }

  //线程thread_visual，每100ms从sim获取link位姿
  static std::thread thread_visual([&] {
    running = true;
    const double delta_t = 0.001;
    while (!num_contacts && running && !threadStop) {
      auto start = std::chrono::steady_clock::now();
      //从sire获取link位姿
      sim.GetLinkPQ(link_pq);
      //通过mainwindow.cpp中槽函数传递link_pm给occview.cpp中的setLinkPm()
      emit updateLinkPQ(link_pq);
      //计算是否碰撞
       collision.CalCollision(link_pq, num_contacts);
      //if (num_contacts) {
      //   for (int i = 0; i < 7 * 7; ++i) {
      //    std::cout << link_pq[i] << " ";
      //   }
      //   std::cout << std::endl;
      //}
      std::this_thread::sleep_until(start + std::chrono::duration<double, std::milli>(delta_t * 1000));
    }
  });
  //分离线程thread_visual
  if (thread_visual.joinable()) {
    thread_visual.detach();
  }
}

void threadSimulation::ThreadStart() {
  QMutexLocker locker(&m_lock);
  threadStop = false;
}

void threadSimulation::ThreadStop() {
  QMutexLocker locker(&m_lock);
  threadStop = true;
}

threadSimulation::~threadSimulation() {
  running = false;// set to stop detached thread
}
