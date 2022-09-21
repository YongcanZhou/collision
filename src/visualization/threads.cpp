#include "threads.h"

threadSimulation::threadSimulation(QThread *parent)
	: QThread(parent)
{

}

void threadSimulation::run()
{
	while (!threadStop)
	{
		QThread::msleep(100);
		angle += 0.05;
		if (angle > 3.14) {
			angle = 0.0;
		}
		emit updataAngle(angle);
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

}














