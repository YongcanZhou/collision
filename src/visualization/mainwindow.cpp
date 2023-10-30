#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <Eigen/StdVector>

#define PCL_NO_PRECOMPILE

#include <QMetaType>
#include <QVector>
#include <array>
#include <boost/array.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>//变换矩阵类
#include <pcl/visualization/pcl_visualizer.h>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow) {
  ui->setupUi(this);
  occWidget = new OccView(this);
  this->setWindowTitle(QString("KAANH"));
  this->setWindowIcon(QIcon(":/Kaanh.jpg"));

  ui->menu_3->setIcon(QIcon(":/themes/dark/import.svg"));
  ui->actionRbtJointImport->setIcon(QIcon(":/themes/dark/import.svg"));
  ui->actionRbtImport->setIcon(QIcon(":/themes/dark/import.svg"));
  ui->actionWopImport->setIcon(QIcon(":/themes/dark/import.svg"));
  ui->actionToolImport->setIcon(QIcon(":/themes/dark/import.svg"));
  ui->actionSTLImport->setIcon(QIcon(":/themes/dark/import.svg"));

  ui->actionExport->setIcon(QIcon(":/themes/dark/export.svg"));
  ui->actionClose->setIcon(QIcon(":/themes/dark/stop.svg"));

  ui->splitter->setVisible(true);
  ui->splitter->setChildrenCollapsible(false);
  ui->splitter->setStretchFactor(0, 1);
  ui->splitter->setStretchFactor(1, 3);
  ui->splitter->setStyleSheet("QSplitter::handle{background-color: black}");
  ui->splitter->setHandleWidth(3);
  ui->splitter_2->setStyleSheet("QSplitter::handle{background-color: black}");
  ui->splitter_2->setHandleWidth(3);

  ui->comboBoxDocuments->addItem("Doc1");
  ui->comboBoxDocuments->addItem("Doc2");
  ui->comboBoxDocuments->addItem("Doc3");

  threadSimulation *threadsim = new threadSimulation();


  /*****occTabWidgetPage1******/
  /*****occTabWidgetPage1******/
  /*****occTabWidgetPage1******/
  /*****occTabWidgetPage1******/

  QVBoxLayout *layout = new QVBoxLayout(this);
  QHBoxLayout *layout01 = new QHBoxLayout(this);
  QHBoxLayout *layout02 = new QHBoxLayout(this);
  QHBoxLayout *layout03 = new QHBoxLayout(this);
  QHBoxLayout *layout04 = new QHBoxLayout(this);

  QHBoxLayout* layouts[4];



  auto buttonFitAll = Ui::createViewBtn(this, QIcon(":/themes/dark/expand.svg"), tr("Fit All"));
  QSpacerItem *hSpacer01 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
  QSpacerItem *hSpacer02 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
  QSpacerItem *hSpacer03 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

  auto buttonNewToolCoordinate = new QPushButton(this);
  buttonNewToolCoordinate->setText(tr("NewToolCoordinate"));
  auto buttonNewPartCoordinate = new QPushButton(this);
  buttonNewPartCoordinate->setText(tr("NewPartCoordinate"));
  auto buttonAnaminationStart = new QPushButton(this);
  buttonAnaminationStart->setText(tr("AnaminationStart"));
  auto buttonAnaminationStop = new QPushButton(this);
  buttonAnaminationStop->setText(tr("AnaminationStop"));

  auto buttonX = new QPushButton(this);
  buttonX->setText(tr("X+"));
  auto buttonY = new QPushButton(this);
  buttonY->setText(tr("Y+"));
  auto buttonZ = new QPushButton(this);
  buttonZ->setText(tr("Z+"));
  auto buttonRx = new QPushButton(this);
  buttonRx->setText(tr("Rx+"));
  auto buttonRy = new QPushButton(this);
  buttonRy->setText(tr("Ry+"));
  auto buttonRz = new QPushButton(this);
  buttonRz->setText(tr("Rz+"));

  auto buttonBackX = new QPushButton(this);
  buttonBackX->setText(tr("X-"));
  auto buttonBackY = new QPushButton(this);
  buttonBackY->setText(tr("Y-"));
  auto buttonBackZ = new QPushButton(this);
  buttonBackZ->setText(tr("Z-"));
  auto buttonBackRx = new QPushButton(this);
  buttonBackRx->setText(tr("Rx-"));
  auto buttonBackRy = new QPushButton(this);
  buttonBackRy->setText(tr("Ry-"));
  auto buttonBackRz = new QPushButton(this);
  buttonBackRz->setText(tr("Rz-"));

  auto buttonAxis01Forward = new QPushButton(this);
  buttonAxis01Forward->setText(tr("R1+"));
  auto buttonAxis02Forward = new QPushButton(this);
  buttonAxis02Forward->setText(tr("R2+"));
  auto buttonAxis03Forward = new QPushButton(this);
  buttonAxis03Forward->setText(tr("R3+"));
  auto buttonAxis04Forward = new QPushButton(this);
  buttonAxis04Forward->setText(tr("R4+"));
  auto buttonAxis05Forward = new QPushButton(this);
  buttonAxis05Forward->setText(tr("R5+"));
  auto buttonAxis06Forward = new QPushButton(this);
  buttonAxis06Forward->setText(tr("R6+"));


  auto buttonAxis01Backward = new QPushButton(this);
  buttonAxis01Backward->setText(tr("R1-"));
  auto buttonAxis02Backward = new QPushButton(this);
  buttonAxis02Backward->setText(tr("R2-"));
  auto buttonAxis03Backward = new QPushButton(this);
  buttonAxis03Backward->setText(tr("R3-"));
  auto buttonAxis04Backward = new QPushButton(this);
  buttonAxis04Backward->setText(tr("R4-"));
  auto buttonAxis05Backward = new QPushButton(this);
  buttonAxis05Backward->setText(tr("R5-"));
  auto buttonAxis06Backward = new QPushButton(this);
  buttonAxis06Backward->setText(tr("R6-"));

  struct ButtonCreationData {
    QIcon icon;
    QString text;
    V3d_TypeOfOrientation proj;
  };
  const ButtonCreationData btnCreationData[] = {
          {QIcon(":/themes/dark/view-iso.svg"), tr("Isometric"), V3d_XposYnegZpos},
          {QIcon(":/themes/dark/view-back.svg"), tr("Back"), V3d_Ypos},
          {QIcon(":/themes/dark/view-front.svg"), tr("Front"), V3d_Yneg},
          {QIcon(":/themes/dark/view-left.svg"), tr("Left"), V3d_Xneg},
          {QIcon(":/themes/dark/view-right.svg"), tr("Right"), V3d_Xpos},
          {QIcon(":/themes/dark/view-top.svg"), tr("Top"), V3d_Zpos},
          {QIcon(":/themes/dark/view-bottom.svg"), tr("Bottom"), V3d_Zneg}};

  auto btnViewMenu = Ui::createViewBtn(this, QIcon(":/themes/dark/view-iso.svg"), QString());
  btnViewMenu->setToolTip(btnCreationData[0].text);
  btnViewMenu->setData(static_cast<int>(btnCreationData[0].proj));
  auto menuBtnView = new QMenu(btnViewMenu);
  for (const ButtonCreationData &btnData: btnCreationData) {
    auto action = menuBtnView->addAction(btnData.icon, btnData.text);
    QObject::connect(action, &QAction::triggered, this, [=] {
      occWidget->getView()->SetProj(btnData.proj);
      occWidget->getView()->ZFitAll();
    });
  }


  auto menuFilterHomeButton = new QPushButton(this);
  menuFilterHomeButton->setText("Filters");
  auto menuFilter = new QMenu(menuFilterHomeButton);
  auto action01 = menuFilter->addAction("Point Filter");
  action01->setCheckable(true);
  auto action02 = menuFilter->addAction("Edge Filter");
  action02->setCheckable(true);
  auto action03 = menuFilter->addAction("Wire Filter");
  action03->setCheckable(true);
  auto action04 = menuFilter->addAction("Face Filter");
  action04->setCheckable(true);


  QObject::connect(action01, &QAction::triggered, [=](bool checked) {
    if (checked) {
      occWidget->InstallFilters(TopAbs_ShapeEnum::TopAbs_VERTEX);
    } else {
      occWidget->UninstallFilters(TopAbs_ShapeEnum::TopAbs_VERTEX);
    }
  });
  QObject::connect(action02, &QAction::triggered, this, [=](bool checked) {
    if (checked) {
      occWidget->InstallFilters(TopAbs_ShapeEnum::TopAbs_EDGE);
    } else {
      occWidget->UninstallFilters(TopAbs_ShapeEnum::TopAbs_EDGE);
    }
  });
  QObject::connect(action03, &QAction::triggered, this, [=](bool checked) {
    if (checked) {
      occWidget->InstallFilters(TopAbs_ShapeEnum::TopAbs_WIRE);
    } else {
      occWidget->UninstallFilters(TopAbs_ShapeEnum::TopAbs_WIRE);
    }
  });
  QObject::connect(action04, &QAction::triggered, this, [=](bool checked) {
    if (checked) {
      occWidget->InstallFilters(TopAbs_ShapeEnum::TopAbs_FACE);
    } else {
      occWidget->UninstallFilters(TopAbs_ShapeEnum::TopAbs_FACE);
    }
  });

  qDebug() << "TopAbs_ShapeEnum::TopAbs_EDGE" << TopAbs_ShapeEnum::TopAbs_EDGE;
  qDebug() << "TopAbs_ShapeEnum::TopAbs_VERTEX" << TopAbs_ShapeEnum::TopAbs_VERTEX;


  QProgressBar *bar = new QProgressBar(this);
  bar->setRange(0, 100);              //设置进度条最小值和最大值(取值范围)
  bar->setMinimum(0);                 //设置进度条最小值
  bar->setMaximum(100);               //设置进度条最大值
  bar->reset();                       //让进度条重新回到开始
  bar->setOrientation(Qt::Horizontal);//水平方向
  bar->setAlignment(Qt::AlignVCenter);// 对齐方式
  bar->setTextVisible(false);         //隐藏进度条文本
  //bar->setFixedSize(258,5);   //进度条固定大小
  bar->setInvertedAppearance(false);//true:反方向  false:正方向
  bar->setVisible(true);            //false:隐藏进度条  true:显示进度条
  QObject::connect(occWidget, &OccView::sendimportValueSigal, [=](int value) {
    bar->setValue(value);
  });


  layout01->addWidget(buttonFitAll);
  layout01->addWidget(btnViewMenu);
  layout01->addWidget(menuFilterHomeButton);
  layout01->addWidget(buttonNewToolCoordinate);
  layout01->addWidget(buttonNewPartCoordinate);
  layout01->addWidget(buttonAnaminationStart);
  layout01->addWidget(buttonAnaminationStop);
  layout01->addItem(hSpacer01);
  layout01->addWidget(bar);


  occStackWidgetsPage1 = new QStackedWidget(ui->occTabWidgetPage1);
  QHBoxLayout *layoutStack001 = new QHBoxLayout(this);
  layoutStack001->addWidget(occWidget);

  occStackWidgetsPage1->addWidget(occWidget);
  layout02->addWidget(occStackWidgetsPage1);


  auto docptr = Application::instance()->newDocument();
  docptr->setName("Doc1");
  struct occWidgets occwidgetstruct;
  occwidgetstruct.widget01 = occWidget;
  clampWidget *clampview = new clampWidget(this);
  QVTKOpenGLStereoWidget *pointsview = new QVTKOpenGLStereoWidget(this);
  occwidgetstruct.widget02 = pointsview;
  occwidgetstruct.widget03 = clampview;
  DocumentsPtr.push_back({docptr, occwidgetstruct});
  Doc1_treeitem = new QTreeWidgetItem();
  Doc2_treeitem = new QTreeWidgetItem();
  Doc3_treeitem = new QTreeWidgetItem();
  //treeitem->setText(0,DocumentsPtr.begin()->first->name);
  //ui->treeWidget->addTopLevelItem(treeitem);

  Doc1_treeitem->setText(0, "Doc1");
  Doc1_Robot_treeitem = new QTreeWidgetItem(Doc1_treeitem);
  Doc1_Robot_treeitem->setText(0, "Robot");
  Doc1_Robot_treeitem->setCheckState(0, Qt::Checked);

  Doc2_treeitem->setText(0, "Doc2");
  Doc2_Robot_treeitem = new QTreeWidgetItem(Doc2_treeitem);
  Doc2_Robot_treeitem->setText(0, "Robot");
  Doc2_Robot_treeitem->setCheckState(0, Qt::Checked);

  Doc3_treeitem->setText(0, "Doc3");
  Doc3_Robot_treeitem = new QTreeWidgetItem(Doc3_treeitem);
  Doc3_Robot_treeitem->setText(0, "Robot");
  Doc3_Robot_treeitem->setCheckState(0, Qt::Checked);


  Doc1_Tool_treeitem = new QTreeWidgetItem(Doc1_treeitem);
  Doc1_Tool_treeitem->setText(0, "Tool");
  Doc1_Tool_treeitem->setCheckState(0, Qt::Checked);

  Doc2_Tool_treeitem = new QTreeWidgetItem(Doc2_treeitem);
  Doc2_Tool_treeitem->setText(0, "Tool");
  Doc2_Tool_treeitem->setCheckState(0, Qt::Checked);

  Doc3_Tool_treeitem = new QTreeWidgetItem(Doc3_treeitem);
  Doc3_Tool_treeitem->setText(0, "Tool");
  Doc3_Tool_treeitem->setCheckState(0, Qt::Checked);


  Doc1_Part_treeitem = new QTreeWidgetItem(Doc1_treeitem);
  Doc1_Part_treeitem->setText(0, "Part");
  Doc1_Part_treeitem->setCheckState(0, Qt::Checked);

  Doc2_Part_treeitem = new QTreeWidgetItem(Doc2_treeitem);
  Doc2_Part_treeitem->setText(0, "Part");
  Doc2_Part_treeitem->setCheckState(0, Qt::Checked);

  Doc3_Part_treeitem = new QTreeWidgetItem(Doc3_treeitem);
  Doc3_Part_treeitem->setText(0, "Part");
  Doc3_Part_treeitem->setCheckState(0, Qt::Checked);

  Doc1_Joint1_treeitem = new QTreeWidgetItem(Doc1_Robot_treeitem);
  Doc1_Joint1_treeitem->setText(0, "Joint1");
  Doc1_Joint1_treeitem->setCheckState(0, Qt::Checked);

  Doc2_Joint1_treeitem = new QTreeWidgetItem(Doc2_Robot_treeitem);
  Doc2_Joint1_treeitem->setText(0, "Joint1");
  Doc2_Joint1_treeitem->setCheckState(0, Qt::Checked);

  Doc3_Joint1_treeitem = new QTreeWidgetItem(Doc3_Robot_treeitem);
  Doc3_Joint1_treeitem->setText(0, "Joint1");
  Doc3_Joint1_treeitem->setCheckState(0, Qt::Checked);

  Doc1_Joint2_treeitem = new QTreeWidgetItem(Doc1_Robot_treeitem);
  Doc1_Joint2_treeitem->setText(0, "Joint2");
  Doc1_Joint2_treeitem->setCheckState(0, Qt::Checked);

  Doc2_Joint2_treeitem = new QTreeWidgetItem(Doc2_Robot_treeitem);
  Doc2_Joint2_treeitem->setText(0, "Joint2");
  Doc2_Joint2_treeitem->setCheckState(0, Qt::Checked);

  Doc3_Joint2_treeitem = new QTreeWidgetItem(Doc3_Robot_treeitem);
  Doc3_Joint2_treeitem->setText(0, "Joint2");
  Doc3_Joint2_treeitem->setCheckState(0, Qt::Checked);

  Doc1_Joint3_treeitem = new QTreeWidgetItem(Doc1_Robot_treeitem);
  Doc1_Joint3_treeitem->setText(0, "Joint3");
  Doc1_Joint3_treeitem->setCheckState(0, Qt::Checked);

  Doc2_Joint3_treeitem = new QTreeWidgetItem(Doc2_Robot_treeitem);
  Doc2_Joint3_treeitem->setText(0, "Joint3");
  Doc2_Joint3_treeitem->setCheckState(0, Qt::Checked);

  Doc3_Joint3_treeitem = new QTreeWidgetItem(Doc3_Robot_treeitem);
  Doc3_Joint3_treeitem->setText(0, "Joint3");
  Doc3_Joint3_treeitem->setCheckState(0, Qt::Checked);


  Doc1_Joint4_treeitem = new QTreeWidgetItem(Doc1_Robot_treeitem);
  Doc1_Joint4_treeitem->setText(0, "Joint4");
  Doc1_Joint4_treeitem->setCheckState(0, Qt::Checked);

  Doc2_Joint4_treeitem = new QTreeWidgetItem(Doc2_Robot_treeitem);
  Doc2_Joint4_treeitem->setText(0, "Joint4");
  Doc2_Joint4_treeitem->setCheckState(0, Qt::Checked);

  Doc3_Joint4_treeitem = new QTreeWidgetItem(Doc3_Robot_treeitem);
  Doc3_Joint4_treeitem->setText(0, "Joint4");
  Doc3_Joint4_treeitem->setCheckState(0, Qt::Checked);


  Doc1_Joint5_treeitem = new QTreeWidgetItem(Doc1_Robot_treeitem);
  Doc1_Joint5_treeitem->setText(0, "Joint5");
  Doc1_Joint5_treeitem->setCheckState(0, Qt::Checked);

  Doc2_Joint5_treeitem = new QTreeWidgetItem(Doc2_Robot_treeitem);
  Doc2_Joint5_treeitem->setText(0, "Joint5");
  Doc2_Joint5_treeitem->setCheckState(0, Qt::Checked);

  Doc3_Joint5_treeitem = new QTreeWidgetItem(Doc3_Robot_treeitem);
  Doc3_Joint5_treeitem->setText(0, "Joint5");
  Doc3_Joint5_treeitem->setCheckState(0, Qt::Checked);


  Doc1_Joint6_treeitem = new QTreeWidgetItem(Doc1_Robot_treeitem);
  Doc1_Joint6_treeitem->setText(0, "Joint6");
  Doc1_Joint6_treeitem->setCheckState(0, Qt::Checked);

  Doc2_Joint6_treeitem = new QTreeWidgetItem(Doc2_Robot_treeitem);
  Doc2_Joint6_treeitem->setText(0, "Joint6");
  Doc2_Joint6_treeitem->setCheckState(0, Qt::Checked);

  Doc3_Joint6_treeitem = new QTreeWidgetItem(Doc3_Robot_treeitem);
  Doc3_Joint6_treeitem->setText(0, "Joint6");
  Doc3_Joint6_treeitem->setCheckState(0, Qt::Checked);


  Doc1_ExternalToolEnable_item = new QTreeWidgetItem(Doc1_treeitem);
  Doc1_ExternalToolEnable_item->setText(0, "ExternalToolEnable");
  Doc1_ExternalToolEnable_item->setCheckState(0, Qt::Checked);

  Doc2_ExternalToolEnable_item = new QTreeWidgetItem(Doc2_treeitem);
  Doc2_ExternalToolEnable_item->setText(0, "ExternalToolEnable");
  Doc2_ExternalToolEnable_item->setCheckState(0, Qt::Checked);

  Doc3_ExternalToolEnable_item = new QTreeWidgetItem(Doc3_treeitem);
  Doc3_ExternalToolEnable_item->setText(0, "ExternalToolEnable");
  Doc3_ExternalToolEnable_item->setCheckState(0, Qt::Checked);


  ui->treeWidget->addTopLevelItem(Doc1_treeitem);
  ui->treeWidget->addTopLevelItem(Doc2_treeitem);
  ui->treeWidget->addTopLevelItem(Doc3_treeitem);
  connect(ui->treeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this,
          SLOT(TreeItemClicked(QTreeWidgetItem *, int)));


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
  ui->occTabWidgetPage1->setLayout(layout);

  QObject::connect(buttonFitAll, &Ui::ButtonFlat::clicked, this, [&] { occWidget->getView()->FitAll(); });
  QObject::connect(btnViewMenu, &Ui::ButtonFlat::clicked, this, [=] {
    menuBtnView->popup(btnViewMenu->mapToGlobal({0, btnViewMenu->height()}));
  });
  QObject::connect(menuFilterHomeButton, &QPushButton::clicked, this, [=] {
    menuFilter->popup(menuFilterHomeButton->mapToGlobal({0, menuFilter->height()}));
  });

  //    QObject::connect(occWidget, &OccView::mouseMovedSignal, [=](const QPoint& pos2d) {
  //        //guiDoc->graphicsScene()->highlightAt(pos2d, widget->guiDocument()->v3dView());
  auto selector = occWidget->getContext()->MainSelector();
  //        selector->Pick(pos2d.x(), pos2d.y(), occWidget->getView());
  //        const gp_Pnt pos3d =
  //                selector->NbPicked() > 0 ?
  //                    selector->PickedPoint(1) :
  //                    Ui::V3dView_to3dPosition(occWidget->getView(), pos2d.x(), pos2d.y());
  //        qDebug()<<QString::number(pos3d.X(), 'f', 3)<<","<<QString::number(pos3d.Y(), 'f', 3)<<","<<QString::number(pos3d.Z(), 'f', 3);
  //    });

  QObject::connect(buttonNewToolCoordinate, &QPushButton::clicked, this, [=] {
    if (!occWidget->getNewToolCoordinate()) {
      QPalette pal = buttonNewToolCoordinate->palette();
      pal.setColor(QPalette::ButtonText, Qt::green);
      buttonNewToolCoordinate->setPalette(pal);
      occWidget->getNewToolCoordinate() = true;
      occWidget->initNewToolCoordinate();
    } else {
      QPalette pal = buttonNewToolCoordinate->palette();
      pal.setColor(QPalette::ButtonText, Qt::gray);
      buttonNewToolCoordinate->setPalette(pal);
      occWidget->getNewToolCoordinate() = false;
    }
  });


  QObject::connect(buttonNewPartCoordinate, &QPushButton::clicked, this, [=] {
    if (!occWidget->getNewPartCoordinate()) {
      QPalette pal = buttonNewPartCoordinate->palette();
      pal.setColor(QPalette::ButtonText, Qt::green);
      buttonNewPartCoordinate->setPalette(pal);
      occWidget->getNewPartCoordinate() = true;
      occWidget->initNewPartCoordinate();
    } else {
      QPalette pal = buttonNewPartCoordinate->palette();
      pal.setColor(QPalette::ButtonText, Qt::gray);
      buttonNewPartCoordinate->setPalette(pal);
      occWidget->getNewPartCoordinate() = false;
    }
  });

  QObject::connect(buttonAnaminationStart, &QPushButton::clicked, this, [=] {
    threadsim->ThreadStart();
    threadsim->start();
    buttonAnaminationStart->setEnabled(false);
  });

  QObject::connect(buttonAnaminationStop, &QPushButton::clicked, this, [=] {
    threadsim->ThreadStop();
    buttonAnaminationStart->setEnabled(true);
    buttonAnaminationStop->setEnabled(true);
  });

  QObject::connect(buttonAxis01Forward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis01MoveForward();
  });
  QObject::connect(buttonAxis02Forward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis02MoveForward();
  });
  QObject::connect(buttonAxis03Forward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis03MoveForward();
  });
  QObject::connect(buttonAxis04Forward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis04MoveForward();
  });
  QObject::connect(buttonAxis05Forward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis05MoveForward();
  });
  QObject::connect(buttonAxis06Forward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis06MoveForward();
  });


  QObject::connect(buttonAxis01Backward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis01MoveBackward();
  });
  QObject::connect(buttonAxis02Backward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis02MoveBackward();
  });
  QObject::connect(buttonAxis03Backward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis03MoveBackward();
  });
  QObject::connect(buttonAxis04Backward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis04MoveBackward();
  });
  QObject::connect(buttonAxis05Backward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis05MoveBackward();
  });
  QObject::connect(buttonAxis06Backward, &QPushButton::clicked, this, [&] {
    occWidget->ButtonAxis06MoveBackward();
  });


  /*****occTabWidgetPage2******/
  /*****occTabWidgetPage2******/
  /*****occTabWidgetPage2******/
  /*****occTabWidgetPage2******/


  QVBoxLayout *layout0200 = new QVBoxLayout(this);
  QHBoxLayout *layout0201 = new QHBoxLayout(this);
  QHBoxLayout *layout0202 = new QHBoxLayout(this);
  QHBoxLayout *layout0203 = new QHBoxLayout(this);

  auto button0201test = new QPushButton(this);
  button0201test->setText(tr("test"));
  QSpacerItem *hSpacer0201 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
  layout0201->addItem(hSpacer0201);
  layout0201->addWidget(button0201test);

  auto button0202test = new QPushButton(this);
  button0202test->setText(tr("test"));
  QSpacerItem *hSpacer0202 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
  layout0202->addItem(hSpacer0202);
  layout0202->addWidget(button0202test);

  occStackWidgetsPage2 = new QStackedWidget(this);
  //QHBoxLayout* layoutStack002 = new QHBoxLayout(this);
  //occStackWidgetsPage2->setLayout(layoutStack002);
  occStackWidgetsPage2->addWidget(pointsview);
  layout0203->addWidget(occStackWidgetsPage2);

  layout0200->addLayout(layout0201);
  layout0200->addLayout(layout0203);
  layout0200->addLayout(layout0202);
  ui->occTabWidgetPage2->setLayout(layout0200);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(
                                              pcl::PointCloud<pcl::PointXYZ>()),
                                      cloud_icp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>());
  // cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  // cloud_icp.reset(new pcl::PointCloud<pcl::PointXYZ>);
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  // viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = std::make_shared<pcl::visualization::PCLVisualizer>(
          pcl::visualization::PCLVisualizer("viewer", false));
#if VTK_MAJOR_VERSION > 8
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
  pointsview->setRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(pointsview->interactor(), pointsview->renderWindow());
#endif
  char strfilepath[256] = "./bunny.pcd";
  pcl::io::loadPCDFile(strfilepath, *cloud);
  //viewer->addPointCloud(cloud, "cloud");


  double theta = M_PI / 20;// 设置旋转弧度的角度
  Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
  transformation_matrix(0, 0) = cos(theta);
  transformation_matrix(0, 1) = -sin(theta);
  transformation_matrix(1, 0) = sin(theta);
  transformation_matrix(1, 1) = cos(theta);

  //// 设置平移矩阵
  transformation_matrix(0, 3) = 0.0;
  transformation_matrix(1, 3) = 0.0;
  transformation_matrix(2, 3) = 4.0;
  // 执行初始变换
  pcl::transformPointCloud(*cloud, *cloud_icp, transformation_matrix);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(cloud, 250, 0, 0);
  //设置源点云的颜色为红色
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(cloud_icp, 0, 250, 0);
  //目标点


  int v1(1);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("viewer_01", 10, 10, "v1 text", v1);
  viewer->addPointCloud(cloud, sources_cloud_color, "sample cloud1",
                        v1);
  //添加点云pointCloudPtr，其处理结果rgb，添加到视图中，sample cloud1为点云的名字，v1代表是添加到哪个视图窗口（pcl中可设置多窗口模式）

  //第二个窗口显示内容进行设定
  int v2(2);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  // 功能尚不明确，仅仅知道：四个数值代表X最小值，Y最小值，X最大值，Y最大值，显示窗口的ID号
  viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
  //设置v2的背景颜色  背景颜色的设置范围被归化到0-1之间
  viewer->addText("viewer_02", 10, 10, "v2 text", v2);
  //功能尚不明确？
  viewer->addPointCloud(cloud_icp, target_cloud_color, "sample cloud2", v1);
  //上面一行代码含义：pointCloudPtr为真正的点云，将真正的点云上色，然后把结果存放在：single_color中；


  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud);
  icp.setInputTarget(cloud_icp);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final;
  // 存储配准变换源点云后的点云
  Final.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_cloud_color(Final, 0, 250, 0);
  //目标点


  icp.align(*Final);

  viewer->addPointCloud(Final, final_cloud_color, "final_cloud_color", v2);
  viewer->addPointCloud(cloud_icp, target_cloud_color, "sample cloud222", v2);

  qDebug() << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();
  qDebug() << icp.getFinalTransformation()(0, 0);
  qDebug() << "testtesttest";

#if VTK_MAJOR_VERSION > 8
  pointsview->renderWindow()->Render();
#else
  pointsview->update();
#endif


  /*****occTabWidgetPage3******/
  /*****occTabWidgetPage3******/
  /*****occTabWidgetPage3******/
  /*****occTabWidgetPage3******/


  QVBoxLayout *layout0300 = new QVBoxLayout(this);
  QHBoxLayout *layout0301 = new QHBoxLayout(this);
  QHBoxLayout *layout0302 = new QHBoxLayout(this);
  QHBoxLayout *layout0303 = new QHBoxLayout(this);

  auto button0301test = new QPushButton(this);
  button0301test->setText(tr("test"));
  QSpacerItem *hSpacer0301 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
  layout0301->addItem(hSpacer0301);
  layout0301->addWidget(button0301test);

  auto button0302test = new QPushButton(this);
  button0302test->setText(tr("test"));
  QSpacerItem *hSpacer0302 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
  layout0302->addItem(hSpacer0302);
  layout0302->addWidget(button0302test);

  occStackWidgetsPage3 = new QStackedWidget(this);
  //QHBoxLayout* layoutStack003 = new QHBoxLayout(this);
  //occStackWidgetsPage3->setLayout(layoutStack003);
  occStackWidgetsPage3->addWidget(clampview);
  layout0303->addWidget(occStackWidgetsPage3);

  layout0300->addLayout(layout0301);
  layout0300->addLayout(layout0303);
  layout0300->addLayout(layout0302);
  ui->occTabWidgetPage3->setLayout(layout0300);


  /*****tabWidgetPage1******/
  /*****tabWidgetPage1******/
  /*****tabWidgetPage1******/
  /*****tabWidgetPage1******/
  auto buttonRobotMoveSim = new QPushButton(this);
  buttonRobotMoveSim->setText(tr("RobotMoveSim"));
  auto buttonPartMoveSim = new QPushButton(this);
  buttonPartMoveSim->setText(tr("PartMoveSim"));
  auto buttontoolTrihedronDisplay = new QPushButton(this);
  buttontoolTrihedronDisplay->setText(tr("toolTrihedronDisplay"));
  auto buttonRobotHome = new QPushButton(this);
  buttonRobotHome->setText(tr("RobotHome"));
  QVBoxLayout *layout05 = new QVBoxLayout(this);
  QSpacerItem *hSpacer04 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);


  layout05->addItem(hSpacer04);
  layout05->addWidget(buttontoolTrihedronDisplay);
  layout05->addWidget(buttonPartMoveSim);
  layout05->addWidget(buttonRobotMoveSim);
  layout05->addWidget(buttonRobotHome);
  ui->tabWidgetPage1->setLayout(layout05);


  QObject::connect(buttonRobotMoveSim, &QPushButton::clicked, this, [&] {
    occWidget->RobotMoveSim();
  });

  QObject::connect(buttonPartMoveSim, &QPushButton::clicked, this, [&] {
    occWidget->PartMoveSim();
  });

  QObject::connect(buttontoolTrihedronDisplay, &QPushButton::clicked, this, [&] {
    occWidget->toolTrihedronDisplay();
  });
  QObject::connect(buttonRobotHome, &QPushButton::clicked, this, [&] {
    occWidget->RobotBackHome();
  });


  /*****tabWidgetPage2******/
  /*****tabWidgetPage2******/
  /*****tabWidgetPage2******/
  /*****tabWidgetPage2******/

  EditPartXCoor = new QLineEdit(this);
  EditPartYCoor = new QLineEdit(this);
  EditPartZCoor = new QLineEdit(this);
  EditPartRXCoor = new QLineEdit(this);
  EditPartRYCoor = new QLineEdit(this);
  EditPartRZCoor = new QLineEdit(this);

  EditPartXCoor->setPlaceholderText(tr("X:"));
  EditPartYCoor->setPlaceholderText(tr("Y:"));
  EditPartZCoor->setPlaceholderText(tr("Z:"));
  EditPartRXCoor->setPlaceholderText(tr("RX:"));
  EditPartRYCoor->setPlaceholderText(tr("RY:"));
  EditPartRZCoor->setPlaceholderText(tr("RZ:"));


  auto buttonPartCoorOK = new QPushButton(this);
  buttonPartCoorOK->setText(tr("PartCoordinateOK"));
  QVBoxLayout *layout06 = new QVBoxLayout(this);
  QSpacerItem *hSpacer05 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
  layout06->addWidget(EditPartXCoor);
  layout06->addWidget(EditPartYCoor);
  layout06->addWidget(EditPartZCoor);
  layout06->addWidget(EditPartRXCoor);
  layout06->addWidget(EditPartRYCoor);
  layout06->addWidget(EditPartRZCoor);
  layout06->addWidget(buttonPartCoorOK);
  layout06->addItem(hSpacer05);
  ui->tabWidgetPage2->setLayout(layout06);


  QObject::connect(buttonPartCoorOK, &QPushButton::clicked, this, [&] {
    Ui::EularCoor eularcoor;
    eularcoor.x = EditPartXCoor->text().toDouble();
    eularcoor.y = EditPartYCoor->text().toDouble();
    eularcoor.z = EditPartZCoor->text().toDouble();
    eularcoor.rx = EditPartRXCoor->text().toDouble() * PI_OCC / 180;
    eularcoor.ry = EditPartRYCoor->text().toDouble() * PI_OCC / 180;
    eularcoor.rz = EditPartRZCoor->text().toDouble() * PI_OCC / 180;
    occWidget->getPartCoor() = eularcoor;
    occWidget->ButtonPartCoorOK();
  });

  QObject::connect(occWidget, &OccView::NewToolCoordinateCompleteSigal, this, [=] {
    QPalette pal = buttonNewToolCoordinate->palette();
    pal.setColor(QPalette::ButtonText, Qt::gray);
    buttonNewToolCoordinate->setPalette(pal);
    buttonNewToolCoordinate->setEnabled(true);
    occWidget->getNewToolCoordinate() = false;


    EditToolXCoor->setText(QString::number(occWidget->getToolCoor().x, 'f', 3));
    EditToolYCoor->setText(QString::number(occWidget->getToolCoor().y, 'f', 3));
    EditToolZCoor->setText(QString::number(occWidget->getToolCoor().z, 'f', 3));
    EditToolRXCoor->setText(QString::number(occWidget->getToolCoor().rx * 180 / PI_OCC, 'f', 3));
    EditToolRYCoor->setText(QString::number(occWidget->getToolCoor().ry * 180 / PI_OCC, 'f', 3));
    EditToolRZCoor->setText(QString::number(occWidget->getToolCoor().rz * 180 / PI_OCC, 'f', 3));
  });

  QObject::connect(occWidget, &OccView::NewPartCoordinateCompleteSigal, this, [=] {
    QPalette pal = buttonNewPartCoordinate->palette();
    pal.setColor(QPalette::ButtonText, Qt::gray);
    buttonNewPartCoordinate->setPalette(pal);
    buttonNewPartCoordinate->setEnabled(true);
    occWidget->getNewPartCoordinate() = false;

    EditPartXCoor->setText(QString::number(occWidget->getPartCoor().x, 'f', 3));
    EditPartYCoor->setText(QString::number(occWidget->getPartCoor().y, 'f', 3));
    EditPartZCoor->setText(QString::number(occWidget->getPartCoor().z, 'f', 3));
    EditPartRXCoor->setText(QString::number(occWidget->getPartCoor().rx * 180 / PI_OCC, 'f', 3));
    EditPartRYCoor->setText(QString::number(occWidget->getPartCoor().ry * 180 / PI_OCC, 'f', 3));
    EditPartRZCoor->setText(QString::number(occWidget->getPartCoor().rz * 180 / PI_OCC, 'f', 3));
  });


  /*****tabWidgetPage3******/
  /*****tabWidgetPage3******/
  /*****tabWidgetPage3******/
  /*****tabWidgetPage3******/

  EditToolXCoor = new QLineEdit(this);
  EditToolYCoor = new QLineEdit(this);
  EditToolZCoor = new QLineEdit(this);
  EditToolRXCoor = new QLineEdit(this);
  EditToolRYCoor = new QLineEdit(this);
  EditToolRZCoor = new QLineEdit(this);

  EditToolXCoor->setPlaceholderText(tr("X:"));
  EditToolYCoor->setPlaceholderText(tr("Y:"));
  EditToolZCoor->setPlaceholderText(tr("Z:"));
  EditToolRXCoor->setPlaceholderText(tr("RX:"));
  EditToolRYCoor->setPlaceholderText(tr("RY:"));
  EditToolRZCoor->setPlaceholderText(tr("RZ:"));

  auto buttonToolCoorOK = new QPushButton(this);
  buttonToolCoorOK->setText(tr("ToolCoordinateOK"));
  QVBoxLayout *layout07 = new QVBoxLayout(this);
  QSpacerItem *hSpacer06 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
  layout07->addWidget(EditToolXCoor);
  layout07->addWidget(EditToolYCoor);
  layout07->addWidget(EditToolZCoor);
  layout07->addWidget(EditToolRXCoor);
  layout07->addWidget(EditToolRYCoor);
  layout07->addWidget(EditToolRZCoor);
  layout07->addWidget(buttonToolCoorOK);
  layout07->addItem(hSpacer06);
  ui->tabWidgetPage3->setLayout(layout07);

  QObject::connect(buttonToolCoorOK, &QPushButton::clicked, this, [&] {
    Ui::EularCoor eularcoor;
    eularcoor.x = EditToolXCoor->text().toDouble();
    eularcoor.y = EditToolYCoor->text().toDouble();
    eularcoor.z = EditToolZCoor->text().toDouble();
    eularcoor.rx = EditToolRXCoor->text().toDouble() * PI_OCC / 180;
    eularcoor.ry = EditToolRYCoor->text().toDouble() * PI_OCC / 180;
    eularcoor.rz = EditToolRZCoor->text().toDouble() * PI_OCC / 180;
    occWidget->getToolCoor() = eularcoor;
    occWidget->ButtonToolCoorOK();
  });


  /*****tabWidgetPage4******/
  /*****tabWidgetPage4******/
  /*****tabWidgetPage4******/
  /*****tabWidgetPage4******/

  auto buttonFirstCurve = new QPushButton(this);
  buttonFirstCurve->setText(tr("buttonFirstCurve"));
  auto plainTextFirstCurve = new QPlainTextEdit(this);

  auto buttonsecondCurve = new QPushButton(this);
  buttonsecondCurve->setText(tr("secondFirstCurve"));
  auto plainTextSecondCurve = new QPlainTextEdit(this);

  auto buttonCalPlain = new QPushButton(this);
  buttonCalPlain->setText(tr("buttonCalPlain"));
  auto plainTextCalPlain = new QPlainTextEdit(this);

  auto buttonStartCal = new QPushButton(this);
  buttonStartCal->setText(tr("buttonStartCal"));


  auto buttonStartSim = new QPushButton(this);
  buttonStartSim->setText(tr("buttonStartSim"));

  QVBoxLayout *layout08 = new QVBoxLayout(this);
  QSpacerItem *hSpacer07 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

  layout08->addWidget(buttonFirstCurve);
  layout08->addWidget(plainTextFirstCurve);
  layout08->addWidget(buttonsecondCurve);
  layout08->addWidget(plainTextSecondCurve);
  layout08->addWidget(buttonCalPlain);
  layout08->addWidget(plainTextCalPlain);
  layout08->addWidget(buttonStartCal);
  layout08->addWidget(buttonStartSim);
  layout08->addItem(hSpacer07);

  ui->tabWidgetPage4->setLayout(layout08);

  //发送选择指令
  QObject::connect(buttonFirstCurve, &QPushButton::clicked, this, [=] {
    occWidget->ButtonFirstCurve();
    if (buttonFirstCurve->isEnabled()) {
      buttonFirstCurve->setEnabled(false);
    }
  });
  QObject::connect(buttonsecondCurve, &QPushButton::clicked, this, [=] {
    occWidget->ButtonSecondCurve();
    if (buttonsecondCurve->isEnabled()) {
      buttonsecondCurve->setEnabled(false);
    }
  });
  QObject::connect(buttonCalPlain, &QPushButton::clicked, this, [=] {
    occWidget->ButtonPlainSelect();
    if (!buttonStartCal->isEnabled()) {
      buttonStartCal->setEnabled(false);
    }
  });

  //显示选择后的文字
  QObject::connect(occWidget, &OccView::firstCurveCompleteSigal, this, [=] {
    plainTextFirstCurve->appendPlainText(occWidget->PairfirstCurve.back().first);
    if (!buttonFirstCurve->isEnabled()) {
      buttonFirstCurve->setEnabled(true);
    }
  });
  QObject::connect(occWidget, &OccView::secondCurveCompleteSigal, this, [=] {
    plainTextSecondCurve->appendPlainText(occWidget->PairsecondCurve.back().first);
    if (!buttonsecondCurve->isEnabled()) {
      buttonsecondCurve->setEnabled(true);
    }
  });
  QObject::connect(occWidget, &OccView::faceSelectCompleteSigal, this, [=] {
    plainTextCalPlain->appendPlainText(occWidget->PairPlains.back().first);
    if (!buttonStartCal->isEnabled()) {
      buttonStartCal->setEnabled(true);
    }
  });

  //开始计算指令
  QObject::connect(buttonStartCal, &QPushButton::clicked, this, [=] {
    occWidget->ButtonPointsCal();
  });

  //  QObject::connect(buttonStartSim, &QPushButton::clicked, this, [=] {
  //      SLOT(occWidget->setLinkPQ(std::array<double, 7 * 7>) );
  //  });

  /*****anyTest******/
  /*****anyTest******/
  /*****anyTest******/
  /*****anyTest******/

  qRegisterMetaType<std::array<double, 7 * 7>>("std::array<double, 7 * 7>");

  qRegisterMetaType<std::array<double, 7 >>("std::array<double, 7 >");

  qRegisterMetaType<std::vector<std::array<double, 6>>>("std::vector<std::array<double, 6>>");

  // qRegisterMetaType<std::array<double, 7 * 16>>("std::array<double, 7 * 16>");
  //QObject::connect(threadsim, SIGNAL(updataAngle(double)), occWidget, SLOT(setAngle(double)));
  //QObject::connect(threadsim, SIGNAL(updateLinkPm(std::array<double, 7 * 16>)), occWidget, SLOT(setLinkPm(std::array<double, 7 * 16>)));

  QObject::connect(threadsim, SIGNAL(updateLinkPQ(std::array<double, 7 * 7>)), occWidget,
                   SLOT(setLinkPQ(std::array<double, 7 * 7>)));
  QObject::connect(threadsim, SIGNAL(updateSpherePQ(std::array<double, 7 >)), occWidget,
                   SLOT(setSpherePQ(std::array<double, 7 >)));

  //  QObject::connect(occWidget,SIGNAL(setTrackPoints(std::vector<std::array<double, 6>>)),threadsim,
  //                                   SLOT(GetTrackPoints(std::vector<std::array<double, 6>> )) );
  anyTest();


  //thread_visual
  update_time = new QTimer();
  QObject::connect(update_time, SIGNAL(timeout()), this, SLOT(time_update()));
  update_time->start(1);//1ms秒钟后启动
}

MainWindow::~MainWindow() {
  delete occWidget;
  delete ui;
}

//导入机器人 robot
void MainWindow::on_actionRbtImport_triggered() {
  QFileDialog *fileDialog = new QFileDialog(this);
  fileDialog->setWindowTitle(tr("Open Image"));
  fileDialog->setDirectory(".");
  fileDialog->setNameFilters(QStringList("(*.STEP)"));
  QString path;
  if (fileDialog->exec() == QDialog::Accepted) {
    path = fileDialog->selectedFiles()[0];
    QMessageBox::information(nullptr, tr("Path"), tr("You selected") + path);
    occWidget->robotPath = path;
    //occWidget->loadDisplayRobotWhole();
    //occWidget->ocaf->load(ui->treeWidget);
    occWidget->ReadFile(path, DocumentsPtr.begin()->first);

    //diff
    bool m_isMergeXdeReferredShapeOn = true;
    auto buildWidgetItem = [&](QTreeWidgetItem *treeItem, const Assemly_Tree &modelTree) {
      std::unordered_map<NodeId, QTreeWidgetItem *> mapNodeIdToTreeItem;
      std::unordered_set<NodeId> setReferenceNodeId;

      Ui::traverseTree(1, modelTree, [&](NodeId itNodeId) {
        const NodeId nodeParentId = modelTree.nodeParent(itNodeId);
        auto itParentFound = mapNodeIdToTreeItem.find(nodeParentId);
        QTreeWidgetItem *guiParentNode =
                itParentFound != mapNodeIdToTreeItem.end() ? itParentFound->second : treeItem;
        if (m_isMergeXdeReferredShapeOn) {
          const TDF_Label &nodeLabel = modelTree.nodeData(itNodeId).label;
          auto guiNode = new QTreeWidgetItem(guiParentNode);
          QString guiNodeText = Ui::cafutils::to_QString(Ui::cafutils::labelAttrStdName(nodeLabel));
          NodeId guiNodeId = itNodeId;
          if (setReferenceNodeId.find(nodeParentId) != setReferenceNodeId.cend()) {
            // const TDF_Label& refLabel = modelTree.nodeData(nodeParentId).label;
            //guiNodeText = this->referenceItemText(refLabel, nodeLabel);
            guiNodeId = nodeParentId;
            if (!guiParentNode)
              mapNodeIdToTreeItem[nodeParentId] = guiNode;
          }

          guiNode->setText(0, guiNodeText);
          //WidgetModelTree::setDocumentTreeNode(guiNode, DocumentTreeNode(doc, guiNodeId));
          const QIcon icon = Ui::cafutils::shapeIcon(nodeLabel);
          if (!icon.isNull())
            guiNode->setIcon(0, icon);
          guiNode->setFlags(guiNode->flags() | Qt::ItemIsUserCheckable);
          guiNode->setCheckState(0, Qt::Checked);
          mapNodeIdToTreeItem[itNodeId] = guiNode;
        }
      });
      //return mapNodeIdToTreeItem.find(0)->second;
    };

    bool ok{false};

    //auto treeitem=ui->treeWidget->findItems(DocumentsPtr.begin()->first->name,ok);
    //        buildWidgetItem(treeitem,DocumentsPtr.begin()->first->docTree);
    //        ui->treeWidget->addTopLevelItem(treeitem);

  } else {
    QMessageBox::information(nullptr, tr("Path"), tr("You didn't select any files."));
  }

}

//加载工件 workpiece
void MainWindow::on_actionWopImport_triggered() {
  qDebug() << "on_actionWopImport_triggered";
  QFileDialog *fileDialog = new QFileDialog(this);
  fileDialog->setWindowTitle(tr("Open Image"));
  fileDialog->setDirectory(".");
  fileDialog->setNameFilters(QStringList("(*.STEP)"));
  QString path;
  if (fileDialog->exec() == QDialog::Accepted) {
    path = fileDialog->selectedFiles()[0];
    QMessageBox::information(nullptr, tr("Path"), tr("You selected") + path);
    occWidget->workpiecePath = path;
    qDebug() << "wop path:" << occWidget->workpiecePath;
    occWidget->loadDisplayWorkpiece();
  } else {
    QMessageBox::information(nullptr, tr("Path"), tr("You didn't select any files."));
  }

}


//导入机器人关节robot joint
void MainWindow::on_actionRbtJointImport_triggered() {
  qDebug() << "on_actionRbtJointImport_triggered";
  QFileDialog *fileDialog = new QFileDialog(this);
  fileDialog->setWindowTitle(tr("Open Image"));
  fileDialog->setDirectory(".");
  fileDialog->setNameFilters(QStringList("(*.STEP)"));
  QString path;
  if (fileDialog->exec() == QDialog::Accepted) {
    path = fileDialog->selectedFiles()[0];
    QMessageBox::information(nullptr, tr("Path"), tr("You selected") + path);
    occWidget->robotPath = path;
    qDebug() << "path:" << occWidget->robotPath;
    occWidget->loadDisplayRobotJoints();
  } else {
    QMessageBox::information(nullptr, tr("Path"), tr("You didn't select any files."));
  }

}

//导入工具
void MainWindow::on_actionToolImport_triggered() {
  qDebug() << "on_actionToolImport_triggered";
  QFileDialog *fileDialog = new QFileDialog(this);
  fileDialog->setWindowTitle(tr("Open Image"));
  fileDialog->setDirectory(".");
  fileDialog->setNameFilters(QStringList("(*.STEP)"));
  QString path;
  if (fileDialog->exec() == QDialog::Accepted) {
    path = fileDialog->selectedFiles()[0];
    QMessageBox::information(nullptr, tr("Path"), tr("You selected") + path);
    occWidget->toolPath = path;
    qDebug() << "tool path:" << occWidget->toolPath;
    occWidget->loadDisplayTool();
  } else {
    QMessageBox::information(nullptr, tr("Path"), tr("You didn't select any files."));
  }

}
//导入点云
void MainWindow::on_actionSTLImport_triggered() {
  qDebug() << "on_actionSTLImport_triggered";
  QFileDialog *fileDialog = new QFileDialog(this);
  fileDialog->setWindowTitle(tr("Open STL"));
  fileDialog->setDirectory(".");
  fileDialog->setNameFilters(QStringList("(*.STL)"));
  QString path;
  if (fileDialog->exec() == QDialog::Accepted) {
    path = fileDialog->selectedFiles()[0];
    QMessageBox::information(nullptr, tr("Path"), tr("You selected") + path);
    occWidget->stlPath = path;
    qDebug() << "path:" << occWidget->stlPath;
    occWidget->loadDisplaySTL();
  } else {
    QMessageBox::information(nullptr, tr("Path"), tr("You didn't select any files."));
  }

}

void MainWindow::anyTest() {
  //double thetas[6]{93.14*PI/180,-62.68*PI/180,108.27*PI/180,-135.56*PI/180,-66.46*PI/180,15.59*PI/180};
  //Ui::UR5RobotForwardCal(thetas);

  ///******UR5正解*********/
  //{
  //    double d[6 + 1] = { 0, 0.089159,0,0,0.10915,0.09465,0.0823 };//第0个不用
  //    double a[6] = { 0,-0.42500,-0.39225,0,0,0 };
  //    double alpha[6] = { 1.570796, 0, 0, 1.570796, -1.570796, 0 };
  //    //double theta_input[7]{0,93.14*PI/180,-62.68*PI/180,108.27*PI/180,-135.56*PI/180,-66.46*PI/180,15.59*PI/180};
  //    double theta_input[7]{0,-PI/2,-PI/2,0,-PI/2,0,0};
  //    Eigen::Matrix4d T[6 + 1];//为了和theta对应，0不用
  //    for (int i = 1; i <= 6; i++)
  //    {
  //        T[i](0, 0) = cos(theta_input[i]);
  //        T[i](0, 1) = -sin(theta_input[i]) * cos(alpha[i - 1]);
  //        T[i](0, 2) = sin(theta_input[i]) * sin(alpha[i - 1]);
  //        T[i](0, 3) = a[i - 1] * cos(theta_input[i]);
  //        T[i](1, 0) = sin(theta_input[i]);
  //        T[i](1, 1) = cos(theta_input[i]) * cos(alpha[i - 1]);
  //        T[i](1, 2) = -cos(theta_input[i]) * sin(alpha[i - 1]);
  //        T[i](1, 3) = a[i - 1] * sin(theta_input[i]);
  //        T[i](2, 0) = 0;
  //        T[i](2, 1) = sin(alpha[i - 1]);
  //        T[i](2, 2) = cos(alpha[i - 1]);
  //        T[i](2, 3) = d[i];
  //        T[i](3, 0) = 0;
  //        T[i](3, 1) = 0;
  //        T[i](3, 2) = 0;
  //        T[i](3, 3) = 1;
  //    }
  //    Eigen::Matrix4d T0 = T[1] * T[2] * T[3] * T[4] * T[5] * T[6];
  //    qDebug() << "angle:" << T0(0, 0) << " " << T0(0, 1) << " " << T0(0, 2) << " px = " << T0(0, 3) << endl;
  //    qDebug()  << "angle:" << T0(1, 0) << " " << T0(1, 1) << " " << T0(1, 2) << " py = " << T0(1, 3) << endl;
  //    qDebug()  << "angle:" << T0(2, 0) << " " << T0(2, 1) << " " << T0(2, 2) << " pz = " << T0(2, 3) << endl;
  //    qDebug()  << "length:" << T0(0, 3) << " " << T0(1, 3) << " " << T0(2, 3) << endl;
  //}


  /******UR5逆解*********/
  //    {
  //        Eigen::Matrix4d input;
  //        double output[6];

  //        double pe[6]{0.17269,-0.55555,0.11106,3.025,0.395,2.901};
  //        Ui::s_pe2pm(pe,input,"123");
  //        Ui::UR5RobotInverseCal(input,output);
  //    }

  /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  char strfilepath[256] = "C:/Users/taye709/Desktop/pcl_test/rabbit.pcd";
  pcl::io::loadPCDFile(strfilepath, *cloud);

  viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  viewer->addPointCloud(cloud, "cloud");



  ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
  ui->qvtkWidget->update();*/
}

Handle(Document) MainWindow::currentDoc() {
  return nullptr;
}

void MainWindow::TreeItemClicked(QTreeWidgetItem *item, int data) {
  qDebug() << "TreeItemClicked";
  if (item == nullptr) {
    return;
  }
  if (item == Doc1_Joint1_treeitem) {
    if (Doc1_Joint1_treeitem->checkState(0) == Qt::Checked) {
      occWidget->setVisiable(OccView::nameState::joint1, true);
    } else if (Doc1_Joint1_treeitem->checkState(0) == Qt::Unchecked) {
      occWidget->setVisiable(OccView::nameState::joint1, false);
    }
  }

  if (item == Doc1_Part_treeitem) {
    if (Doc1_Part_treeitem->checkState(0) == Qt::Checked) {
      occWidget->setVisiable(OccView::nameState::part, true);
    } else if (Doc1_Part_treeitem->checkState(0) == Qt::Unchecked) {
      occWidget->setVisiable(OccView::nameState::part, false);
    }
  }

  if (item == Doc1_Tool_treeitem) {
    if (Doc1_Tool_treeitem->checkState(0) == Qt::Checked) {
      occWidget->setVisiable(OccView::nameState::tool, true);
    } else if (Doc1_Tool_treeitem->checkState(0) == Qt::Unchecked) {
      occWidget->setVisiable(OccView::nameState::tool, false);
    }
  }

  if (item == Doc1_ExternalToolEnable_item) {
    if (Doc1_ExternalToolEnable_item->checkState(0) == Qt::Checked) {
      occWidget->setExternalToolEnable(true);
    } else if (Doc1_Tool_treeitem->checkState(0) == Qt::Unchecked) {
      occWidget->setExternalToolEnable(false);
    }
  }
}


void MainWindow::on_actionNewDoc_triggered() {
  bool ok{false}, index{false};
  QString text = QInputDialog::getText(nullptr, "Input doc name", "Doc name:", QLineEdit::Normal, "", &ok);
  for (auto k = DocumentsPtr.begin(); k != DocumentsPtr.end(); k++) {
    if (k->first->name == text) {
      index = true;
      break;
    }
  }
  if (index) {
    QMessageBox::information(nullptr, tr("Error"), tr("name error"));
  } else {
    occStackWidgetsPage1->addWidget(occWidget);

    ui->comboBoxDocuments->addItem(text);
    struct occWidgets occwidgetstruct;
    auto docptr = Application::instance()->newDocument();
    docptr->setName(text);
    occwidgetstruct.widget01 = occWidget;

    QVTKOpenGLStereoWidget *pointsview = new QVTKOpenGLStereoWidget(this);
    clampWidget *clampview = new clampWidget(this);
    //occStackWidgetsPage2->addWidget(pointsview);
    //occStackWidgetsPage3->addWidget(clampview);
    occwidgetstruct.widget02 = pointsview;
    occwidgetstruct.widget03 = clampview;
    DocumentsPtr.push_back({docptr, occwidgetstruct});
    QTreeWidgetItem *item = new QTreeWidgetItem();
    item->setText(0, text);
    ui->treeWidget->addTopLevelItem(item);


    QLabel *pointlabel = new QLabel(this);
    QLabel *clamplabel = new QLabel(this);
    pointlabel->setText("pointlabel");
    clamplabel->setText("clamplabel");

    QHBoxLayout *ttt = new QHBoxLayout(this);
    QHBoxLayout *ddd = new QHBoxLayout(this);

    ttt->addWidget(pointlabel);
    ddd->addWidget(clamplabel);

    pointsview->setLayout(ttt);
    clampview->setLayout(ddd);
  }
}

void MainWindow::on_comboBoxDocuments_currentIndexChanged(int index) {
  auto name = ui->comboBoxDocuments->currentText();
  for (auto k = DocumentsPtr.begin(); k != DocumentsPtr.end(); k++) {
    if (k->first->name == name) {
      this->occStackWidgetsPage1->setCurrentWidget(k->second.widget01);
      this->occStackWidgetsPage2->setCurrentWidget(k->second.widget02);
      this->occStackWidgetsPage3->setCurrentWidget(k->second.widget03);
      break;
    }
  }
}

void MainWindow::time_update() {
  occWidget->visual_update();
}
