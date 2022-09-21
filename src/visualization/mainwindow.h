#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <unordered_map>
#include <unordered_set>
#include <string_view>

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QSpacerItem>
#include <QStackedWidget>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QStylePainter>
#include <QInputDialog>
#include <QLabel>
#include <QProgressBar>
#include <QVTKOpenGLStereoWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
// #include <QVTKWidget.h>

#include "occview.h"
#include "clampwidget.h"
#include "pointswidget.h"
#include "general.h"
#include "documents.h"
#include "application.h"

#include <QTimer>


namespace Ui {
    class MainWindow;
}


struct occWidgets{
    QWidget* widget01;
    QVTKOpenGLStereoWidget* widget02;
    QWidget* widget03;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_actionWopImport_triggered();
    void on_actionRbtImport_triggered();
    void on_actionRbtJointImport_triggered();
    void on_actionToolImport_triggered();
    void on_actionSTLImport_triggered();
    void on_actionNewDoc_triggered();

    void on_comboBoxDocuments_currentIndexChanged(int index);

    void time_update();

private:
    Ui::MainWindow *ui;
    OccView *occWidget;
    QLineEdit* EditPartXCoor;
    QLineEdit* EditPartYCoor;
    QLineEdit* EditPartZCoor;
    QLineEdit* EditPartRXCoor;
    QLineEdit* EditPartRYCoor;
    QLineEdit* EditPartRZCoor;

    QLineEdit* EditToolXCoor;
    QLineEdit* EditToolYCoor;
    QLineEdit* EditToolZCoor;
    QLineEdit* EditToolRXCoor;
    QLineEdit* EditToolRYCoor;
    QLineEdit* EditToolRZCoor;

private:
    void anyTest();
    Handle(Document) currentDoc();
    std::vector<std::pair<Handle(Document),occWidgets>> DocumentsPtr;
    QStackedWidget* occStackWidgetsPage1;
    QStackedWidget* occStackWidgetsPage2;
    QStackedWidget* occStackWidgetsPage3;


	QTreeWidgetItem* Doc1_Robot_treeitem, *Doc1_Joint1_treeitem, *Doc1_Joint2_treeitem, *Doc1_Joint3_treeitem, *Doc1_Joint4_treeitem, *Doc1_Joint5_treeitem, *Doc1_Joint6_treeitem;
	QTreeWidgetItem* Doc2_Robot_treeitem, *Doc2_Joint1_treeitem, *Doc2_Joint2_treeitem, *Doc2_Joint3_treeitem, *Doc2_Joint4_treeitem, *Doc2_Joint5_treeitem, *Doc2_Joint6_treeitem;
	QTreeWidgetItem* Doc3_Robot_treeitem, *Doc3_Joint1_treeitem, *Doc3_Joint2_treeitem, *Doc3_Joint3_treeitem, *Doc3_Joint4_treeitem, *Doc3_Joint5_treeitem, *Doc3_Joint6_treeitem;
	QTreeWidgetItem* Doc1_Tool_treeitem, *Doc2_Tool_treeitem, *Doc3_Tool_treeitem, *Doc1_Part_treeitem, *Doc2_Part_treeitem, *Doc3_Part_treeitem;

	QTreeWidgetItem* Doc1_treeitem, *Doc2_treeitem, *Doc3_treeitem;
	QTreeWidgetItem* Doc1_ExternalToolEnable_item, *Doc2_ExternalToolEnable_item, *Doc3_ExternalToolEnable_item;

    //thread_visual
    QTimer* update_time;


private slots:
	void TreeItemClicked(QTreeWidgetItem* item, int data);
};




class TabWidget : public QTabWidget
{
public:
    explicit TabWidget(QWidget *parent=nullptr):QTabWidget(parent)
    {
        //        this->addTab(new QWidget(this),QStringLiteral("轨迹校正"));
        //        this->addTab(new QWidget(this),QStringLiteral("点云处理"));
        //        this->addTab(new QWidget(this),QStringLiteral("夹持校正"));
        //        this->addTab(new QWidget(this),QStringLiteral("在线仿真"));

        //设置背景黑色
        //        QPalette pal(occWidget->palette());
        //        pal.setColor(QPalette::Background, Qt::black);
        //        occWidget->setAutoFillBackground(true);
        //        occWidget->setPalette(pal);
        this->setTabPosition(QTabWidget::West);
        this->setTabShape(QTabWidget::Triangular);
    }
    void displayWidgets(){
        if(trajectoryWidget!=nullptr&&
                pointsWidget!=nullptr&&
                clampWidget!=nullptr&&
                simulationWidget!=nullptr){
            this->addTab(trajectoryWidget,QStringLiteral("轨迹校正"));
            this->addTab(pointsWidget,QStringLiteral("点云处理"));
            this->addTab(clampWidget,QStringLiteral("夹持校正"));
            this->addTab(simulationWidget,QStringLiteral("在线仿真"));
        }
    }

    void setTrajectoryWidget(QWidget* trajectorywidget){
        trajectoryWidget=trajectorywidget;
    }
    void setPointsWidget(QWidget* pointswidget){
        pointsWidget=pointswidget;
    }
    void setClampWidget(QWidget* clampwidget){
        clampWidget=clampwidget;
    }
    void setSimulationWidget(QWidget* simulationwidget){
        simulationWidget=simulationwidget;
    }

    QWidget* getTrajectoryWidget(){
        return trajectoryWidget;
    }
    QWidget* getPointsWidget(){
        return pointsWidget;
    }
    QWidget* getClampWidget(){
        return clampWidget;
    }
    QWidget* getSimulationWidget(){
        return simulationWidget;
    }

private:
    QWidget* trajectoryWidget;
    QWidget* pointsWidget;
    QWidget* clampWidget;
    QWidget* simulationWidget;
	
};

#endif // MAINWINDOW_H
