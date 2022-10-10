#include "threads.h"
#include <sire.hpp>

threadSimulation::threadSimulation(QThread *parent)
	: QThread(parent)
{
  running = false; // set to stop detached thread(thread_visual)
}

void threadSimulation::run()
{
    num_contacts = 0;
    auto& sim = sire::Simulator::instance("H:/contact/Sire/config/sire.xml");
    auto& collision = sire::Collision::instance("C:/Users/ZHOUYC/Desktop/STL_test");
   
    //开启仿真轨迹规划
    sim.SimPlan();
    //线程thread_visual，每100ms从sim获取link位姿  TODO 线程回收
    static std::thread thread_visual([&] {
        running = true;
        while (num_contacts == 0 && running && !threadStop) {
            //TODO num_contact=0 one time
            auto start = std::chrono::steady_clock::now();
            //从aris_sim获取link位姿
            sim.GetLinkPQ(link_pq);
            //通过mainwindow.cpp中槽函数传递link_pm给occview.cpp中的setLinkPm
            emit updateLinkPQ(link_pq); 

            ////计算是否碰撞
            collision.CalCollision(link_pq, num_contacts);
            std::this_thread::sleep_until(start + std::chrono::milliseconds(100));
        }
        });
    //分离线程thread_visual
    if (thread_visual.joinable()) {
        thread_visual.detach();
    }
}

void threadSimulation::ThreadStart()
{
	QMutexLocker locker(&m_lock);
	threadStop = false;
}

void threadSimulation::ThreadStop()
{
	QMutexLocker locker(&m_lock);
	threadStop = true;
}

 
threadSimulation::~threadSimulation()
{
    running = false; // set to stop detached thread
}














