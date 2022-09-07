#include "src/visualization/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    occWidget=new OccView();
    this->setWindowTitle(QString("KAANH"));
    this->setWindowIcon(QIcon(":/Kaanh.jpg"));

    // ui->actionImport->setIcon(QIcon(":/themes/dark/import.svg"));
    ui->actionRbtImport->setIcon(QIcon(":/themes/dark/import.svg"));
    ui->actionWopImport->setIcon(QIcon(":/themes/dark/import.svg"));
    ui->actionExport->setIcon(QIcon(":/themes/dark/export.svg"));
    ui->actionClose->setIcon(QIcon(":/themes/dark/stop.svg"));

    ui->splitter->setVisible(true);
    ui->splitter->setChildrenCollapsible(false);
    ui->splitter->setStretchFactor(0, 1);
    ui->splitter->setStretchFactor(1, 3);

    QVBoxLayout *layout=new QVBoxLayout(this);
    QHBoxLayout *layout01=new QHBoxLayout(this);
    QHBoxLayout *layout02=new QHBoxLayout(this);
    QHBoxLayout *layout03=new QHBoxLayout(this);
    QHBoxLayout *layout04=new QHBoxLayout(this);


    auto button01=Ui::createViewBtn(this,QIcon(":/themes/dark/expand.svg"), tr("Fit All"));
    QSpacerItem *hSpacer01=new QSpacerItem(40,20,QSizePolicy::Expanding, QSizePolicy::Minimum);
    QSpacerItem *hSpacer02=new QSpacerItem(40,20,QSizePolicy::Expanding, QSizePolicy::Minimum);
    QSpacerItem *hSpacer03=new QSpacerItem(40,20,QSizePolicy::Expanding, QSizePolicy::Minimum);


    auto buttonX=new QPushButton(this);
    buttonX->setText(tr("X+"));
    auto buttonY=new QPushButton(this);
    buttonY->setText(tr("Y+"));
    auto buttonZ=new QPushButton(this);
    buttonZ->setText(tr("Z+"));
    auto buttonRx=new QPushButton(this);
    buttonRx->setText(tr("Rx+"));
    auto buttonRy=new QPushButton(this);
    buttonRy->setText(tr("Ry+"));
    auto buttonRz=new QPushButton(this);
    buttonRz->setText(tr("Rz+"));

    auto buttonBackX=new QPushButton(this);
    buttonBackX->setText(tr("X-"));
    auto buttonBackY=new QPushButton(this);
    buttonBackY->setText(tr("Y-"));
    auto buttonBackZ=new QPushButton(this);
    buttonBackZ->setText(tr("Z-"));
    auto buttonBackRx=new QPushButton(this);
    buttonBackRx->setText(tr("Rx-"));
    auto buttonBackRy=new QPushButton(this);
    buttonBackRy->setText(tr("Ry-"));
    auto buttonBackRz=new QPushButton(this);
    buttonBackRz->setText(tr("Rz-"));

    auto buttonAxis01Forward=new QPushButton(this);
    buttonAxis01Forward->setText(tr("R1+"));
    auto buttonAxis02Forward=new QPushButton(this);
    buttonAxis02Forward->setText(tr("R2+"));
    auto buttonAxis03Forward=new QPushButton(this);
    buttonAxis03Forward->setText(tr("R3+"));
    auto buttonAxis04Forward=new QPushButton(this);
    buttonAxis04Forward->setText(tr("mvj"));
    auto buttonAxis05Forward=new QPushButton(this);
    buttonAxis05Forward->setText(tr("R5+"));
    auto buttonAxis06Forward=new QPushButton(this);
    buttonAxis06Forward->setText(tr("R6+"));


    auto buttonAxis01Backward=new QPushButton(this);
    buttonAxis01Backward->setText(tr("R1-"));
    auto buttonAxis02Backward=new QPushButton(this);
    buttonAxis02Backward->setText(tr("R2-"));
    auto buttonAxis03Backward=new QPushButton(this);
    buttonAxis03Backward->setText(tr("R3-"));
    auto buttonAxis04Backward=new QPushButton(this);
    buttonAxis04Backward->setText(tr("R4-"));
    auto buttonAxis05Backward=new QPushButton(this);
    buttonAxis05Backward->setText(tr("R5-"));
    auto buttonAxis06Backward=new QPushButton(this);
    buttonAxis06Backward->setText(tr("R6-"));



    struct ButtonCreationData {
        QIcon icon;
        QString text;
        V3d_TypeOfOrientation proj;
    };
    const ButtonCreationData btnCreationData[] = {
        { QIcon(":/themes/dark/view-iso.svg"), tr("Isometric"), V3d_XposYnegZpos },
        { QIcon(":/themes/dark/view-back.svg"), tr("Back"), V3d_Ypos },
        { QIcon(":/themes/dark/view-front.svg"), tr("Front"), V3d_Yneg },
        { QIcon(":/themes/dark/view-left.svg"), tr("Left"), V3d_Xneg },
        { QIcon(":/themes/dark/view-right.svg"), tr("Right"), V3d_Xpos },
        { QIcon(":/themes/dark/view-top.svg"), tr("Top"), V3d_Zpos },
        { QIcon(":/themes/dark/view-bottom.svg"), tr("Bottom"), V3d_Zneg }
    };

    auto btnViewMenu = Ui::createViewBtn(this, QIcon(":/themes/dark/view-iso.svg"), QString());
    btnViewMenu->setToolTip(btnCreationData[0].text);
    btnViewMenu->setData(static_cast<int>(btnCreationData[0].proj));
    auto menuBtnView = new QMenu(btnViewMenu);
    for (const ButtonCreationData& btnData : btnCreationData) {
        auto action=menuBtnView->addAction(btnData.icon,btnData.text);
        QObject::connect(action, &QAction::triggered, this, [=]{
            occWidget->getView()->SetProj(btnData.proj);
            occWidget->getView()->ZFitAll();
        });
    }

    layout01->addWidget(button01);
    layout01->addWidget(btnViewMenu);
    layout01->addItem(hSpacer01);

    layout02->addWidget(occWidget);

    layout03->addWidget(buttonX);
    layout03->addWidget(buttonY);
    layout03->addWidget(buttonZ);
    layout03->addWidget(buttonRx);
    layout03->addWidget(buttonRy);
    layout03->addWidget(buttonRz);
    layout03->addWidget(buttonBackX);
    layout03->addWidget(buttonBackY);
    layout03->addWidget(buttonBackZ);
    layout03->addWidget(buttonBackRx);
    layout03->addWidget(buttonBackRy);
    layout03->addWidget(buttonBackRz);
    layout03->addItem(hSpacer02);


    layout04->addWidget(buttonAxis01Forward);
    layout04->addWidget(buttonAxis02Forward);
    layout04->addWidget(buttonAxis03Forward);
    layout04->addWidget(buttonAxis04Forward);
    layout04->addWidget(buttonAxis05Forward);
    layout04->addWidget(buttonAxis06Forward);
    layout04->addWidget(buttonAxis01Backward);
    layout04->addWidget(buttonAxis02Backward);
    layout04->addWidget(buttonAxis03Backward);
    layout04->addWidget(buttonAxis04Backward);
    layout04->addWidget(buttonAxis05Backward);
    layout04->addWidget(buttonAxis06Backward);
    layout04->addItem(hSpacer03);

    layout->addLayout(layout01);
    layout->addLayout(layout02);
    layout->addLayout(layout03);
    layout->addLayout(layout04);
    ui->occWidget->setLayout(layout);

    QObject::connect(button01,&Ui::ButtonFlat::clicked,this,[&]{occWidget->getView()->FitAll();});
    QObject::connect(btnViewMenu, &Ui::ButtonFlat::clicked, this, [=]{
        menuBtnView->popup(btnViewMenu->mapToGlobal({ 0, btnViewMenu->height() }));
    });





    QObject::connect(buttonX,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranXMoveForward();
    });
    QObject::connect(buttonY,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranYMoveForward();
    });
    QObject::connect(buttonZ,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranZMoveForward();
    });
    QObject::connect(buttonRx,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranRXMoveForward();
    });
    QObject::connect(buttonRy,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranRYMoveForward();
    });
    QObject::connect(buttonRz,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranRZMoveForward();
    });


    QObject::connect(buttonBackX,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranXMoveBackward();
    });
    QObject::connect(buttonBackY,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranYMoveBackward();
    });
    QObject::connect(buttonBackZ,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranZMoveBackward();
    });
    QObject::connect(buttonBackRx,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranRXMoveBackward();
    });
    QObject::connect(buttonBackRy,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranRYMoveBackward();
    });
    QObject::connect(buttonBackRz,&QPushButton::clicked,this,[&]{
        occWidget->ButtonTranRZMoveBackward();
    });


    QObject::connect(buttonAxis01Forward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis01MoveForward();
    });
    QObject::connect(buttonAxis02Forward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis02MoveForward();
    });
    QObject::connect(buttonAxis03Forward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis03MoveForward();
    });
    QObject::connect(buttonAxis04Forward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis04MoveForward();
    });
    QObject::connect(buttonAxis05Forward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis05MoveForward();
    });
    QObject::connect(buttonAxis06Forward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis06MoveForward();
    });


    QObject::connect(buttonAxis01Backward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis01MoveBackward();
    });
    QObject::connect(buttonAxis02Backward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis02MoveBackward();
    });
    QObject::connect(buttonAxis03Backward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis03MoveBackward();
    });
    QObject::connect(buttonAxis04Backward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis04MoveBackward();
    });
    QObject::connect(buttonAxis05Backward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis05MoveBackward();
    });
    QObject::connect(buttonAxis06Backward,&QPushButton::clicked,this,[&]{
        occWidget->ButtonAxis06MoveBackward();
    });






    /*****tabWidgetPage1******/
    /*****tabWidgetPage1******/
    /*****tabWidgetPage1******/
    /*****tabWidgetPage1******/
    auto buttonRobotMoveSim=new QPushButton(this);
    buttonRobotMoveSim->setText(tr("RobotMoveSim"));
    auto buttonPartMoveSim=new QPushButton(this);
    buttonPartMoveSim->setText(tr("PartMoveSim"));
    auto buttontoolTrihedronDisplay=new QPushButton(this);
    buttontoolTrihedronDisplay->setText(tr("toolTrihedronDisplay"));
    auto buttonRobotHome=new QPushButton(this);
    buttonRobotHome->setText(tr("RobotHome"));
    QVBoxLayout *layout05=new QVBoxLayout(this);
    QSpacerItem *hSpacer04=new QSpacerItem(40,20,QSizePolicy::Expanding, QSizePolicy::Minimum);


    layout05->addItem(hSpacer04);
    layout05->addWidget(buttontoolTrihedronDisplay);
    layout05->addWidget(buttonPartMoveSim);
    layout05->addWidget(buttonRobotMoveSim);
    layout05->addWidget(buttonRobotHome);
    ui->tabWidgetPage1->setLayout(layout05);

    QObject::connect(buttonRobotHome,&QPushButton::clicked,this,[&]{
        occWidget->RobotBackHome();
    });


    /*****tabWidgetPage2******/
    /*****tabWidgetPage2******/
    /*****tabWidgetPage2******/
    /*****tabWidgetPage2******/

    EditPartXCoor=new QLineEdit(this);
    EditPartYCoor=new QLineEdit(this);
    EditPartZCoor=new QLineEdit(this);
    EditPartRXCoor=new QLineEdit(this);
    EditPartRYCoor=new QLineEdit(this);
    EditPartRZCoor=new QLineEdit(this);

    EditPartXCoor->setPlaceholderText(tr("X:"));
    EditPartYCoor->setPlaceholderText(tr("Y:"));
    EditPartZCoor->setPlaceholderText(tr("Z:"));
    EditPartRXCoor->setPlaceholderText(tr("RX:"));
    EditPartRYCoor->setPlaceholderText(tr("RY:"));
    EditPartRZCoor->setPlaceholderText(tr("RZ:"));


    auto buttonPartCoorOK=new QPushButton(this);
    buttonPartCoorOK->setText(tr("PartCoordinateOK"));
    QVBoxLayout *layout06=new QVBoxLayout(this);
    QSpacerItem *hSpacer05=new QSpacerItem(40,20,QSizePolicy::Expanding, QSizePolicy::Minimum);
    layout06->addWidget(EditPartXCoor);
    layout06->addWidget(EditPartYCoor);
    layout06->addWidget(EditPartZCoor);
    layout06->addWidget(EditPartRXCoor);
    layout06->addWidget(EditPartRYCoor);
    layout06->addWidget(EditPartRZCoor);
    layout06->addWidget(buttonPartCoorOK);
    layout06->addItem(hSpacer05);
    ui->tabWidgetPage2->setLayout(layout06);


    QObject::connect(buttonPartCoorOK,&QPushButton::clicked,this,[&]{
        Ui::EularCoor eularcoor;
        eularcoor.x=EditPartXCoor->text().toDouble();
        eularcoor.y=EditPartYCoor->text().toDouble();
        eularcoor.z=EditPartZCoor->text().toDouble();
        eularcoor.rx=EditPartRXCoor->text().toDouble()*PI/180;
        eularcoor.ry=EditPartRYCoor->text().toDouble()*PI/180;
        eularcoor.rz=EditPartRZCoor->text().toDouble()*PI/180;
        occWidget->getPartCoor()=eularcoor;
    });



    /*****tabWidgetPage3******/
    /*****tabWidgetPage3******/
    /*****tabWidgetPage3******/
    /*****tabWidgetPage3******/

    EditToolXCoor=new QLineEdit(this);
    EditToolYCoor=new QLineEdit(this);
    EditToolZCoor=new QLineEdit(this);
    EditToolRXCoor=new QLineEdit(this);
    EditToolRYCoor=new QLineEdit(this);
    EditToolRZCoor=new QLineEdit(this);

    EditToolXCoor->setPlaceholderText(tr("X:"));
    EditToolYCoor->setPlaceholderText(tr("Y:"));
    EditToolZCoor->setPlaceholderText(tr("Z:"));
    EditToolRXCoor->setPlaceholderText(tr("RX:"));
    EditToolRYCoor->setPlaceholderText(tr("RY:"));
    EditToolRZCoor->setPlaceholderText(tr("RZ:"));

    auto buttonToolCoorOK=new QPushButton(this);
    buttonToolCoorOK->setText(tr("ToolCoordinateOK"));
    QVBoxLayout *layout07=new QVBoxLayout(this);
    QSpacerItem *hSpacer06=new QSpacerItem(40,20,QSizePolicy::Expanding, QSizePolicy::Minimum);
    layout07->addWidget(EditToolXCoor);
    layout07->addWidget(EditToolYCoor);
    layout07->addWidget(EditToolZCoor);
    layout07->addWidget(EditToolRXCoor);
    layout07->addWidget(EditToolRYCoor);
    layout07->addWidget(EditToolRZCoor);
    layout07->addWidget(buttonToolCoorOK);
    layout07->addItem(hSpacer06);
    ui->tabWidgetPage3->setLayout(layout07);




    /*****tabWidgetPage4******/
    /*****tabWidgetPage4******/
    /*****tabWidgetPage4******/
    /*****tabWidgetPage4******/

    auto buttonFirstCurve=new QPushButton(this);
    buttonFirstCurve->setText(tr("buttonFirstCurve"));
    auto plainTextFirstCurve=new QPlainTextEdit(this);

    auto buttonsecondCurve=new QPushButton(this);
    buttonsecondCurve->setText(tr("secondFirstCurve"));
    auto plainTextSecondCurve=new QPlainTextEdit(this);

    auto buttonCalPlain=new QPushButton(this);
    buttonCalPlain->setText(tr("buttonCalPlain"));
    auto plainTextCalPlain=new QPlainTextEdit(this);



    QVBoxLayout *layout08=new QVBoxLayout(this);
    QSpacerItem *hSpacer07=new QSpacerItem(40,20,QSizePolicy::Expanding, QSizePolicy::Minimum);

    layout08->addWidget(buttonFirstCurve);
    layout08->addWidget(plainTextFirstCurve);
    layout08->addWidget(buttonsecondCurve);
    layout08->addWidget(plainTextSecondCurve);
    layout08->addWidget(buttonCalPlain);
    layout08->addWidget(plainTextCalPlain);

    layout08->addItem(hSpacer07);
    ui->tabWidgetPage4->setLayout(layout08);


    /*****anyTest******/
    /*****anyTest******/
    /*****anyTest******/
    /*****anyTest******/

    //thread_visual
    update_time = new QTimer();
    QObject::connect(update_time,SIGNAL(timeout()),this,SLOT(time_update()));
    update_time->start(10); //10m秒钟后启动

}

MainWindow::~MainWindow()
{
    delete occWidget;
    delete ui;
}


void MainWindow::on_actionWopImport_triggered()
{
    QFileDialog *fileDialog = new QFileDialog(this);
    fileDialog->setWindowTitle(tr("Open Image"));
    fileDialog->setDirectory(".");
    fileDialog->setNameFilters(QStringList("(*.STEP)"));
    QString path;
    if(fileDialog->exec() == QDialog::Accepted){
        path = fileDialog->selectedFiles()[0];
        QMessageBox::information(nullptr, tr("Path"), tr("You selected") + path);
        occWidget->workpiecePath=path;
        qDebug()<<"path:"<<occWidget->workpiecePath;
        occWidget->loadDisplayWorkpiece();
    }
    else
    {
        QMessageBox::information(nullptr, tr("Path"), tr("You didn't select any files."));
    }
}

void MainWindow::on_actionRbtImport_triggered()
{
    QFileDialog *fileDialog = new QFileDialog(this);
    fileDialog->setWindowTitle(tr("Open Image"));
    fileDialog->setDirectory(".");
    fileDialog->setNameFilters(QStringList("(*.STEP)"));
    QString path;
    if(fileDialog->exec() == QDialog::Accepted){
        path = fileDialog->selectedFiles()[0];
        QMessageBox::information(nullptr, tr("Path"), tr("You selected") + path);
        occWidget->robotPath=path;
        qDebug()<<"path:"<<occWidget->robotPath;
        occWidget->loadDisplayRobotWhole();
    }
    else
    {
        QMessageBox::information(nullptr, tr("Path"), tr("You didn't select any files."));
    }
}

void MainWindow::on_actionRbtJointImport_triggered()
{
    QFileDialog *fileDialog = new QFileDialog(this);
    fileDialog->setWindowTitle(tr("Open Image"));
    fileDialog->setDirectory(".");
    fileDialog->setNameFilters(QStringList("(*.STEP)"));
    QString path;
    if(fileDialog->exec() == QDialog::Accepted){
        path = fileDialog->selectedFiles()[0];
        QMessageBox::information(nullptr, tr("Path"), tr("You selected") + path);
        occWidget->robotPath=path;
        qDebug()<<"path:"<<occWidget->robotPath;
        occWidget->loadDisplayRobotJoints();
    }
    else
    {
        QMessageBox::information(nullptr, tr("Path"), tr("You didn't select any files."));
    }
}




void MainWindow::on_actionToolImport_triggered()
{
    QFileDialog *fileDialog = new QFileDialog(this);
    fileDialog->setWindowTitle(tr("Open Image"));
    fileDialog->setDirectory(".");
    fileDialog->setNameFilters(QStringList("(*.STEP)"));
    QString path;
    if(fileDialog->exec() == QDialog::Accepted){
        path = fileDialog->selectedFiles()[0];
        QMessageBox::information(nullptr, tr("Path"), tr("You selected") + path);
        occWidget->toolPath=path;
        qDebug()<<"path:"<<occWidget->toolPath;
        occWidget->loadDisplayTool();
    }
    else
    {
        QMessageBox::information(nullptr, tr("Path"), tr("You didn't select any files."));
    }
}


void MainWindow::time_update()
{
  occWidget->visual_update();
}

void MainWindow::on_actionSTLImport_triggered()
{

}
