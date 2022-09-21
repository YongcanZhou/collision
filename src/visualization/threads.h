#pragma once

#include <QObject>
#include <qthread.h>
#include <qmutex.h>

#include "general.h"
#include "occ.h"

class threadSimulation : public QThread
{
	Q_OBJECT

public:
	threadSimulation(QThread *parent=nullptr);
	~threadSimulation();
protected:
	void run();
private:
	double angle{ 0.0 };
	QMutex m_lock;
	bool threadStop{ true };
signals:
	void updataAngle(double angle_);
public:
	void ThreadStart();
	void ThreadStop();

};

class TaskManager;
class TaskProgress;
using TaskJob = std::function<void(TaskProgress*)>;
using TaskId = std::uint64_t;

class Task {
public:
	TaskId id() const { return m_id; }
	const TaskJob& job() const { return m_fn; }
	TaskManager* manager() const { return m_manager; }

private:
	friend class TaskManager;
	TaskId m_id = 0;//任务编号
	TaskJob m_fn;//任务的回调函数
	TaskManager* m_manager = nullptr;
};

class OccProgressIndicator : public QObject, public Message_ProgressIndicator {
	Q_OBJECT
public:
	OccProgressIndicator()
	{
		this->SetScale(0., 100., 1.);
	}
	bool UserBreak() override
	{
		return false;
	}
	bool Show(const bool force) override
	{
		emit updateProgress(this->GetPosition() * 100);
		return false;
	}

signals:
	void updateProgress(int value);

};