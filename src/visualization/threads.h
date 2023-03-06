#pragma once

#include <QObject>
#include <qthread.h>
#include <qmutex.h>

#include "general.h"
#include "occ.h"
#include "occview.h"


class threadSimulation : public QThread
{
	Q_OBJECT

public:
	explicit threadSimulation(QThread *parent=nullptr);
	~threadSimulation() override;
	void ThreadStart();
	void ThreadStop();

protected:
	void run() override;

private:
	double angle{ 0.0 };
	QMutex m_lock;
	bool threadStop{ true };
  std::vector<std::array<double, 6>> TrackPoints;
	std::array<double, 7 * 7> link_pq{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                                    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
	size_t num_contacts{0};
  std::array<double, 7> sphere_pq{0};
	std::atomic_bool running; // set to stop thread
	//std::vector<std::vector<Ui::PointsVector>> CutOver_CAM_pointVecVecs;
   std::vector<std::array<double, 6>> track_points;


signals:
//	void updataAngle(double angle_);
//	void updateLinkPM(std::array<double, 7 * 16> link_pm);
	void updateLinkPQ(std::array<double, 7 * 7> link_pq);
  void updateSpherePQ(std::array<double, 7> );

private slots:
//  void GetTrackPoints(std::vector<std::array<double, 6>> );


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
	TaskId m_id = 0;//??????
	TaskJob m_fn;//???????????
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