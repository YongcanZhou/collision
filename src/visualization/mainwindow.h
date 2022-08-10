#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QSpacerItem>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <src/visualization/occview.h>
#include <src/visualization/general.h>
#include <QTimerEvent>
#include <QDebug>
#include <QTimer>

namespace Ui {
class MainWindow;
}

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

    //thread_visual
    QTimer *update_time;

};

#endif // MAINWINDOW_H
