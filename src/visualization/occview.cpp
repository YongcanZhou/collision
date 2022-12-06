#include "occview.h"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepFeat_SplitShape.hxx>
#include <BRepFill_PipeShell.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAPI_ExtremaCurveCurve.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <GeomFill_Pipe.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <sire.hpp>

// #define ER100
#define XB4

#ifdef UR5
double OccView::Joint01CurrentAngle = -PI / 2;
double OccView::Joint02CurrentAngle = -PI / 2;
double OccView::Joint03CurrentAngle = 0.0;
double OccView::Joint04CurrentAngle = -PI / 2;
double OccView::Joint05CurrentAngle = 0.0;
double OccView::Joint06CurrentAngle = 0.0;

#endif

#ifdef KR16
double OccView::Joint01CurrentAngle = 0.0;
double OccView::Joint02CurrentAngle = 0.0;
double OccView::Joint03CurrentAngle = 0.0;
double OccView::Joint04CurrentAngle = 0.0;
double OccView::Joint05CurrentAngle = 0.0;
double OccView::Joint06CurrentAngle = 0.0;
#endif

#ifdef ER100
double OccView::Joint01CurrentAngle = 0.0;
double OccView::Joint02CurrentAngle = 0.0;
double OccView::Joint03CurrentAngle = 0.0;
double OccView::Joint04CurrentAngle = 0.0;
double OccView::Joint05CurrentAngle = 0.0;
double OccView::Joint06CurrentAngle = 0.0;
#endif

#ifdef XB4
double OccView::Joint01CurrentAngle = 0.0;
double OccView::Joint02CurrentAngle = 0.0;
double OccView::Joint03CurrentAngle = 0.0;
double OccView::Joint04CurrentAngle = 0.0;
double OccView::Joint05CurrentAngle = 0.0;
double OccView::Joint06CurrentAngle = 0.0;
double OccView::Joint01OriginAngle_static = 0.0;
double OccView::Joint02OriginAngle_static = 0.0;
double OccView::Joint03OriginAngle_static = 0.0;
double OccView::Joint04OriginAngle_static = 0.0;
double OccView::Joint05OriginAngle_static = 0.0;
double OccView::Joint06OriginAngle_static = 0.0;
#endif

std::vector<std::array<double, 6>> OccView::trackPoints;
bool OccView::finish_norm{false};

OccView::OccView(QWidget *parent) : QWidget(parent) {

  InitView();
  //InitFilters();
  m_contextMenu = new QMenu(this);//这是右击弹出的菜单Fhome
  m_addAction = new QAction("new", this);
  m_delAction = new QAction("delete", this);
  //给菜单添加菜单项
  m_contextMenu->addAction(m_addAction);
  m_contextMenu->addAction(m_delAction);
  QObject::connect(m_addAction, &QAction::triggered, this, [=] {
    getShape();
    displayNormalVector();
  });
  QObject::connect(m_delAction, &QAction::triggered, this, [=] {
    removeNormalVector();
  });

  KukaAx1 = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
  KukaAx2 = gp_Ax1(gp_Pnt(260, 0, 675), gp_Dir(0, 1, 0));
  KukaAx3 = gp_Ax1(gp_Pnt(260, 0, 1355), gp_Dir(0, 1, 0));
  KukaAx4 = gp_Ax1(gp_Pnt(662, 0, 1320), gp_Dir(1, 0, 0));
  KukaAx5 = gp_Ax1(gp_Pnt(930, 0, 1320), gp_Dir(0, 1, 0));
  KukaAx6 = gp_Ax1(gp_Pnt(1088, 0, 1320), gp_Dir(1, 0, 0));


  ER100_3000Ax1 = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, -1));
  ER100_3000Ax2 = gp_Ax1(gp_Pnt(260, 0, 645.5), gp_Dir(0, 1, 0));
  ER100_3000Ax3 = gp_Ax1(gp_Pnt(260, 0, 645.4 + 1150), gp_Dir(0, 1, 0));
  ER100_3000Ax4 = gp_Ax1(gp_Pnt(260 + 1600, 0, 645.4 + 1150 + 230), gp_Dir(-1, 0, 0));
  ER100_3000Ax5 = gp_Ax1(gp_Pnt(260 + 1600, 0, 645.4 + 1150 + 230), gp_Dir(0, 1, 0));
  ER100_3000Ax6 = gp_Ax1(gp_Pnt(260 + 1600 + 214, 0, 645.4 + 1150 + 230), gp_Dir(-1, 0, 0));


  UR5Ax1 = gp_Ax1(gp_Pnt(0, -70.5, 89.2), gp_Dir(0, -1, 0));
  UR5Ax2 = gp_Ax1(gp_Pnt(0, -141, 154.4), gp_Dir(0, 0, 1));
  UR5Ax3 = gp_Ax1(gp_Pnt(0, -75.8, 508.9), gp_Dir(0, 1, 0));
  UR5Ax4 = gp_Ax1(gp_Pnt(0, -68.8, 901.33), gp_Dir(0, -1, 0));
  UR5Ax5 = gp_Ax1(gp_Pnt(0, -116.3, 946.83), gp_Dir(0, 0, 1));
  UR5Ax6 = gp_Ax1(gp_Pnt(0, -161.8, 994.33), gp_Dir(0, -1, 0));

  XB4Ax1 = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
  XB4Ax2 = gp_Ax1(gp_Pnt(0.040, 0, 0.342), gp_Dir(0, 1, 0));
  XB4Ax3 = gp_Ax1(gp_Pnt(0.040, 0, 0.617), gp_Dir(0, 1, 0));
  XB4Ax4 = gp_Ax1(gp_Pnt(0.040, 0, 0.642), gp_Dir(1, 0, 0));
  XB4Ax5 = gp_Ax1(gp_Pnt(0.320, 0, 0.642), gp_Dir(0, 1, 0));
  XB4Ax6 = gp_Ax1(gp_Pnt(0.393, 0, 0.642), gp_Dir(1, 0, 0));

  //  OccProgressIndicator *indicat = new OccProgressIndicator();
  //  QObject::connect(indicat, SIGNAL(updateProgress(int)), this, SLOT(importValue(int)));

#ifdef UR5
  GeneralAx1 = UR5Ax1;
  GeneralAx2 = UR5Ax2;
  GeneralAx3 = UR5Ax3;
  GeneralAx4 = UR5Ax4;
  GeneralAx5 = UR5Ax5;
  GeneralAx6 = UR5Ax6;
#endif

#ifdef KR16
  GeneralAx1 = KukaAx1;
  GeneralAx2 = KukaAx2;
  GeneralAx3 = KukaAx3;
  GeneralAx4 = KukaAx4;
  GeneralAx5 = KukaAx5;
  GeneralAx6 = KukaAx6;
#endif

#ifdef ER100
  GeneralAx1 = ER100_3000Ax1;
  GeneralAx2 = ER100_3000Ax2;
  GeneralAx3 = ER100_3000Ax3;
  GeneralAx4 = ER100_3000Ax4;
  GeneralAx5 = ER100_3000Ax5;
  GeneralAx6 = ER100_3000Ax6;
#endif

#ifdef XB4
  GeneralAx1 = XB4Ax1;
  GeneralAx2 = XB4Ax2;
  GeneralAx3 = XB4Ax3;
  GeneralAx4 = XB4Ax4;
  GeneralAx5 = XB4Ax5;
  GeneralAx6 = XB4Ax6;
#endif


  //        STEPCAFControl_Reader reader;
  //        reader.SetColorMode(true);
  //        reader.SetNameMode(true);
  //        IFSelect_ReturnStatus status = reader.ReadFile(aFilePath.toUtf8());
  //        Handle(XCAFApp_Application) anApp = XCAFApp_Application::GetApplication();
  //        Handle(TDocStd_Document) doc;
  //        anApp->NewDocument("MDTV-XCAF", doc);
  //        bool yes = reader.Transfer(doc);
  //        if (yes)
  //        {

  //            TDF_Label mainLabel = doc->Main();
  //            Handle(XCAFDoc_ShapeTool) ShapeTool = XCAFDoc_DocumentTool::ShapeTool(mainLabel);
  //            Handle(XCAFDoc_ColorTool) ColorTool = XCAFDoc_DocumentTool::ColorTool(mainLabel);

  //            {
  //                TDF_LabelSequence tdfLabels;
  //                ShapeTool->GetFreeShapes(tdfLabels);   //获取装配体和组件对应名称
  //                int Roots = tdfLabels.Length();
  //                Assemly_Node * RootNode = AssemlyTree.GetRootNode();
  //                for (int index = 1; index <= Roots; index++)
  //                {
  //                    TDF_Label Label = tdfLabels.Value(index);
  //                    Assemly_Data AssemlyData = GetData(ShapeTool, ColorTool, Label, TopLoc_Location());
  //                    Assemly_Node * Node = AssemlyTree.AddNode(RootNode, AssemlyData);
  //                    MakeTree(ShapeTool, ColorTool, Label, TopLoc_Location(), Node, AssemlyTree);
  //                }
  //            }
  //        }
}

OccView::~OccView() noexcept {
  //running = false; // set to stop detached thread
}

void OccView::paintEvent(QPaintEvent *) {
  m_view->Redraw();
}

void OccView::resizeEvent(QResizeEvent *) {
  if (!m_view.IsNull()) {
    m_view->MustBeResized();
  }
}

QPaintEngine *OccView::paintEngine() const {
  return nullptr;
}

void OccView::mousePressEvent(QMouseEvent *event) {
  if (event->buttons() == Qt::LeftButton)//平移
  {
    if (qApp->keyboardModifiers() == Qt::ShiftModifier) {
      m_context->ShiftSelect(true);

      if (getNewToolCoordinate()) {
        newToolCoordinate();
      }

      if (getNewPartCoordinate()) {
        newPartCoordinate();
      }

      if (selectFirstCurve) {
        startSelectFirstCurve();
      }
      if (selectSecondCurve) {
        startSelectSecondCurve();
      }
      if (selectFaces) {
        startSelectPlains();
      }

    } else if (qApp->keyboardModifiers() == Qt::ControlModifier) {
      if (this->getDrawLine()) {
        leftButON = BUTTON_ON;
        myPntStart = Ui::ConvertClickToPoint(event->pos().x(), event->pos().y(), m_view);
      }
    } else {
      m_current_mode = CurAction3d_DynamicPanning;
      m_x_max = event->pos().x();//记录起始X位置
      m_y_max = event->pos().y();//记录起始Y位置
    }
  }

  if (event->buttons() == Qt::MidButton)//旋转
  {
    m_current_mode = CurAction3d_DynamicRotation;
    m_view->StartRotation(event->pos().x(), event->pos().y());
  }

  if (event->buttons() == Qt::RightButton) {
    m_contextMenu->exec(event->globalPos());
  }
}

void OccView::mouseReleaseEvent(QMouseEvent *event) {
  m_current_mode = CurAction3d_Nothing;
  leftButON = BUTTON_OFF;
}

void OccView::mouseMoveEvent(QMouseEvent *event) {
  switch (m_current_mode) {
    case CurAction3d_DynamicPanning:
      //执行平移
      m_view->Pan(event->pos().x() - m_x_max, m_y_max - event->pos().y());
      m_x_max = event->pos().x();
      m_y_max = event->pos().y();
      break;
    case CurAction3d_DynamicRotation:
      //执行旋转
      m_view->Rotation(event->pos().x(), event->pos().y());
      break;
    case CurAction3d_Nothing:
      m_context->MoveTo(event->x(), event->y(), m_view, true);
  }

  const QPoint currPos = this->mapFromGlobal(event->globalPos());
  const QPoint prevPos = m_prevPos;
  m_prevPos = currPos;
  // emit mouseMovedSignal(currPos);

  if (this->getDrawLine() && leftButON) {
    myPntEnd = Ui::ConvertClickToPoint(event->pos().x(), event->pos().y(), m_view);
    Ui::DrawLineByMouse(myPntStart, myPntEnd, PartAISShape, m_context);
  }
}

void OccView::wheelEvent(QWheelEvent *event) {
  m_view->StartZoomAtPoint(event->pos().x(), event->pos().y());
  m_view->ZoomAtPoint(0, 0, event->angleDelta().y(), 0);//执行缩放
}

//mainwindows中加载 机器人 没用到的函数
void OccView::loadDisplayRobotWhole() {
  /*
  load the file
      STEPControl_Reader reader;
      IFSelect_ReturnStatus stat = reader.ReadFile(workpiecePath.toUtf8());
      switch (stat)
      {
      case IFSelect_RetVoid:
          std::cout << "normal execution which created nothing, or no data to process!" << std::endl;
          break;
      case IFSelect_RetDone:
          break;
      case IFSelect_RetError:
          std::cout << "error in command or input data, no execution!" << std::endl;
          break;
      case IFSelect_RetFail:
          std::cout << "execution was run and has failed!" << std::endl;
          break;
      case IFSelect_RetStop:
          std::cout << "indicates end or stop (such as Raise)!" << std::endl;
          break;
      default:
          break;
      }
      if (stat != IFSelect_RetDone)
          return;

      //Selecting STEP entities for translation : The whole file
      //加载文件
      Standard_Integer NbRoots = reader.NbRootsForTransfer();
      Standard_Integer num = reader.TransferRoots();

      //Mapping STEP entities to Open CASCADE Technology shapes
      PartTopoShape = reader.OneShape();
      auto number=reader.NbShapes();
      qDebug()<<"NbRoots:"<< NbRoots;
      qDebug()<<"num:"<< num;
      qDebug()<<"number:"<< number;



      qDebug()<<"position:"<<  PartTopoShape.Location().Transformation().TranslationPart().X()<<","<<  PartTopoShape.Location().Transformation().TranslationPart().Y()<<","<<  PartTopoShape.Location().Transformation().TranslationPart().Z();


      gp_Vec vec(100,110,120);
      gp_Trsf rsf01;
      rsf01.SetTranslationPart(vec);
      TopLoc_Location loc(rsf01);
      PartTopoShape.Move(loc);


      qDebug()<<"position01:"<<  PartTopoShape.Location().Transformation().TranslationPart().X()<<","<<  PartTopoShape.Location().Transformation().TranslationPart().Y()<<","<<  PartTopoShape.Location().Transformation().TranslationPart().Z();
*/

  STEPCAFControl_Reader readercaf;
  readercaf.SetColorMode(true);
  readercaf.SetNameMode(true);
  IFSelect_ReturnStatus status = readercaf.ReadFile(robotPath.toUtf8());
  qDebug() << "status:" << status;
  Handle(XCAFApp_Application) anApp = XCAFApp_Application::GetApplication();
  Handle(TDocStd_Document) doc;

  anApp->NewDocument("MDTV-XCAF", doc);
  qDebug() << "Transfer:"
           << "1";
  bool yes = readercaf.Transfer(doc);
  qDebug() << "Transfer:"
           << "2";
  qDebug() << "Transfer:" << yes;
  ocaf = new OCAFBrowser(doc);
  {
    TDF_Label mainLabel = doc->Main();
    Handle(XCAFDoc_ShapeTool) ShapeTool = XCAFDoc_DocumentTool::ShapeTool(mainLabel);
    Handle(XCAFDoc_ColorTool) ColorTool = XCAFDoc_DocumentTool::ColorTool(mainLabel);
    TDF_LabelSequence tdfLabels;
    ShapeTool->GetFreeShapes(tdfLabels);//获取装配体和组件对应名称
    int Roots = tdfLabels.Length();
    qDebug() << "Roots:" << Roots;


    TDF_Label Label = tdfLabels.Value(1);
    TDF_LabelSequence components;
    ShapeTool->GetComponents(Label, components);
    qDebug() << "components:" << components.Length();

    for (int i = 1; i <= components.Length(); i++) {
      TDF_Label Label00 = components.Value(i);
      auto shape = ShapeTool->GetShape(Label00);
      RobotAISShape[i - 1] = new AIS_Shape(shape);
      m_context->Display(RobotAISShape[i - 1], true);
    }

    TDF_Label testlabel = components.Value(2);

    //        TopLoc_Location ttloc(tttrsf);
    //        auto tttshape=ShapeTool->GetShape(testlabel);
    //        tttshape.Move(ttloc);


    gp_Trsf delta;
    gp_Vec deltavec(0, 0, 0);
    delta.SetTranslationPart(deltavec);
    tttrsf = tttrsf * delta;

    m_context->SetLocation(RobotAISShape[1], delta);
    m_context->Update(RobotAISShape[1], Standard_True);

    Handle_AIS_ConnectedInteractive aiscon = new AIS_ConnectedInteractive;
    aiscon->Connect(RobotAISShape[5], delta);
    aiscon->SetOwner(RobotAISShape[4]);
    if (aiscon->HasConnection()) {
      auto gg = aiscon->HasConnection();
      qDebug() << "aiscon->HasConnection" << gg;
    }
    delta = delta * delta;
    aiscon->Redisplay(true);
    m_context->Update(RobotAISShape[5], Standard_True);
    m_context->SetLocation(RobotAISShape[4], delta);
    m_context->Update(RobotAISShape[4], Standard_True);

    //取消 robot 模型树
    //RobotAISShape[6]->AddChild(RobotAISShape[7]);
    //RobotAISShape[5]->AddChild(RobotAISShape[6]);
    //RobotAISShape[4]->AddChild(RobotAISShape[5]);
    //RobotAISShape[3]->AddChild(RobotAISShape[4]);
    //RobotAISShape[2]->AddChild(RobotAISShape[3]);
    //RobotAISShape[1]->AddChild(RobotAISShape[2]);
    //RobotAISShape[0]->AddChild(RobotAISShape[1]);
  }

  initRobot();
}

void OccView::initRobot() {
#ifdef UR5
  Joint01OriginAngle = getJoint01CurrentAngle();
  Joint02OriginAngle = getJoint02CurrentAngle();
  Joint03OriginAngle = getJoint03CurrentAngle();
  Joint04OriginAngle = getJoint04CurrentAngle();
  Joint05OriginAngle = getJoint05CurrentAngle();
  Joint06OriginAngle = getJoint06CurrentAngle();

  double initAngles[6]{-91.71, -98.96, -126.22, -46.29, 91.39, -1.78};
  for (int k = 0; k < 6; k++) {
    initAngles[k] = initAngles[k] * PI / 180;
  }
  auto angle01 = initAngles[0] - Joint01OriginAngle;
  auto angle02 = initAngles[1] - Joint02OriginAngle;
  auto angle03 = initAngles[2] - Joint03OriginAngle;
  auto angle04 = initAngles[3] - Joint04OriginAngle;
  auto angle05 = initAngles[4] - Joint05OriginAngle;
  auto angle06 = initAngles[5] - Joint06OriginAngle;

  getJoint01CurrentAngle() = angle01 + Joint01OriginAngle;
  getJoint02CurrentAngle() = angle02 + Joint02OriginAngle;
  getJoint03CurrentAngle() = angle03 + Joint03OriginAngle;
  getJoint04CurrentAngle() = angle04 + Joint04OriginAngle;
  getJoint05CurrentAngle() = angle05 + Joint05OriginAngle;
  getJoint06CurrentAngle() = angle06 + Joint06OriginAngle;


  gp_Trsf trans;
  trans.SetRotation(GeneralAx1, angle01);
  m_context->SetLocation(RobotAISShape[1], trans);

  trans.SetRotation(GeneralAx2, angle02);
  m_context->SetLocation(RobotAISShape[2], trans);

  trans.SetRotation(GeneralAx3, angle03);
  m_context->SetLocation(RobotAISShape[3], trans);

  trans.SetRotation(GeneralAx4, angle04);
  m_context->SetLocation(RobotAISShape[4], trans);

  trans.SetRotation(GeneralAx5, angle05);
  m_context->SetLocation(RobotAISShape[5], trans);

  trans.SetRotation(GeneralAx6, angle06);
  m_context->SetLocation(RobotAISShape[6], trans);

  m_context->UpdateCurrentViewer();
#endif// UR5

#ifdef ER100
  Joint01OriginAngle = 0.0;
  Joint02OriginAngle = 0.0;
  Joint03OriginAngle = 0.0;
  Joint04OriginAngle = 0.0;
  Joint05OriginAngle = 0.0;
  Joint06OriginAngle = 0.0;


  /******求解末端tool0坐标矩阵******/
  double jointTool0[6]{Joint01OriginAngle * PI / 180,
                       Joint02OriginAngle * PI / 180,
                       Joint03OriginAngle * PI / 180,
                       Joint04OriginAngle * PI / 180,
                       Joint05OriginAngle * PI / 180,
                       Joint06OriginAngle * PI / 180};

  robot_tool0_matrix = Ui::ESTUN_ER100_3000_MDH_Forward(jointTool0);

  robot_tool0_matrix(0, 3) = robot_tool0_matrix(0, 3) * 1000;
  robot_tool0_matrix(1, 3) = robot_tool0_matrix(1, 3) * 1000;
  robot_tool0_matrix(2, 3) = robot_tool0_matrix(2, 3) * 1000;


  Ui::qDebugMatrix4d(robot_tool0_matrix, "robot_tool0_matrix:");

#endif// ER100

  m_context->Deactivate();
}

void OccView::loadDisplayRobotJoints() {


  STEPControl_Reader reader;
  static int index = 0;
  IFSelect_ReturnStatus stat = reader.ReadFile(robotPath.toUtf8());
  if (stat != IFSelect_RetDone)
    return;
  //加载文件
  Standard_Integer NbRoots = reader.NbRootsForTransfer();
  Standard_Integer num = reader.TransferRoots();
  auto shape = reader.OneShape();
  RobotAISShape[index] = new AIS_Shape(shape);
  m_context->Display(RobotAISShape[index], Standard_True);

  /*index++;
  if (index == 7) {

    RobotAISShape[5]->AddChild(RobotAISShape[6]);
    RobotAISShape[4]->AddChild(RobotAISShape[5]);
    RobotAISShape[3]->AddChild(RobotAISShape[4]);
    RobotAISShape[2]->AddChild(RobotAISShape[3]);
    RobotAISShape[1]->AddChild(RobotAISShape[2]);
    RobotAISShape[0]->AddChild(RobotAISShape[1]);

    index = 0;
  }*/

  m_view->FitAll();
}
//加载工件 workpiece
void OccView::loadDisplayWorkpiece() {
  qDebug() << "loadDisplayWorkpiece";
  STEPControl_Reader reader;
  IFSelect_ReturnStatus stat = reader.ReadFile(workpiecePath.toUtf8());
  switch (stat) {
    case IFSelect_RetVoid:
      std::cout << "normal execution which created nothing, or no data to process!" << std::endl;
      break;
    case IFSelect_RetDone:
      break;
    case IFSelect_RetError:
      std::cout << "error in command or input data, no execution!" << std::endl;
      break;
    case IFSelect_RetFail:
      std::cout << "execution was run and has failed!" << std::endl;
      break;
    case IFSelect_RetStop:
      std::cout << "indicates end or stop (such as Raise)!" << std::endl;
      break;
    default:
      break;
  }
  if (stat != IFSelect_RetDone)
    return;
  //
  //  /*******注册progressbar***********/
  //  OccProgressIndicator *indicat = new OccProgressIndicator();
  //  QObject::connect(indicat, SIGNAL(updateProgress(int)), this, SLOT(importValue(int)));
  //  Handle_XSControl_WorkSession ws = reader.WS();
  //  ws->MapReader()->SetProgress(indicat);

  Standard_Integer NbRoots = reader.NbRootsForTransfer();
  Standard_Integer num = reader.TransferRoots();

  //Mapping STEP entities to Open CASCADE Technology shapes
  PartTopoShape = reader.OneShape();

  auto number = reader.NbShapes();
  qDebug() << "NbRoots:" << NbRoots;
  qDebug() << "num:" << num;
  qDebug() << "number:" << number;
  //qDebug() << "position:" << PartTopoShape.Location().Transformation().TranslationPart().X() << "," << PartTopoShape.Location().Transformation().TranslationPart().Y() << "," << PartTopoShape.Location().Transformation().TranslationPart().Z();
  //gp_Vec vec(200, 200, 700);
  //gp_Trsf rsf01;
  //rsf01.SetTranslationPart(vec);
  //TopLoc_Location loc(rsf01);
  //PartTopoShape.Move(loc);
  //qDebug() << "position01:" << PartTopoShape.Location().Transformation().TranslationPart().X() << "," << PartTopoShape.Location().Transformation().TranslationPart().Y() << "," << PartTopoShape.Location().Transformation().TranslationPart().Z();

  PartAISShape = new AIS_Shape(PartTopoShape);
  if (m_context->HasLocation(PartAISShape)) {
    qDebug() << "m_context->HasLocation(PartAISShape)";
    double rx{0.0}, ry{0.0}, rz{0.0};
    PartTopoShape.Location().Transformation().GetRotation().GetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, rx, ry, rz);
    qDebug() << "rx:" << rx << ","
             << "ry:" << ry << "rz:" << rz;
  }

  double rx{0.0}, ry{0.0}, rz{0.0};
  PartTopoShape.Location().Transformation().GetRotation().GetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, rx, ry, rz);
  qDebug() << "rx:" << rx << ","
           << "ry:" << ry << "rz:" << rz;

  PartAISShape->SetDisplayMode(AIS_Shaded);
  PartAISShape->SetColor(Quantity_NOC_RED);
  m_context->Display(PartAISShape, true);
  m_view->FitAll();
}
//加载工具 Tool
void OccView::loadDisplayTool() {
  STEPControl_Reader reader;
  IFSelect_ReturnStatus stat = reader.ReadFile(toolPath.toUtf8());
  switch (stat) {
    case IFSelect_RetVoid:
      std::cout << "normal execution which created nothing, or no data to process!" << std::endl;
      break;
    case IFSelect_RetDone:
      break;
    case IFSelect_RetError:
      std::cout << "error in command or input data, no execution!" << std::endl;
      break;
    case IFSelect_RetFail:
      std::cout << "execution was run and has failed!" << std::endl;
      break;
    case IFSelect_RetStop:
      std::cout << "indicates end or stop (such as Raise)!" << std::endl;
      break;
    default:
      break;
  }
  if (stat != IFSelect_RetDone)
    return;

  //Selecting STEP entities for translation : The whole file
  //加载文件

  //  /*******注册progressbar***********/
  //  OccProgressIndicator *indicat = new OccProgressIndicator();
  //  QObject::connect(indicat, SIGNAL(updateProgress(int)), this, SLOT(importValue(int)));
  //  Handle_XSControl_WorkSession ws = reader.WS();
  //  ws->MapReader()->SetProgress(indicat);

  Standard_Integer NbRoots = reader.NbRootsForTransfer();
  Standard_Integer num = reader.TransferRoots();

  //Mapping STEP entities to Open CASCADE Technology shapes
  ToolTopoShape = reader.OneShape();

  auto number = reader.NbShapes();
  qDebug() << "NbRoots:" << NbRoots;
  qDebug() << "num:" << num;
  qDebug() << "number:" << number;

  qDebug() << "position:" << ToolTopoShape.Location().Transformation().TranslationPart().X() << ","
           << ToolTopoShape.Location().Transformation().TranslationPart().Y() << ","
           << PartTopoShape.Location().Transformation().TranslationPart().Z();
  qDebug() << "position01:" << ToolTopoShape.Location().Transformation().TranslationPart().X() << ","
           << ToolTopoShape.Location().Transformation().TranslationPart().Y() << ","
           << PartTopoShape.Location().Transformation().TranslationPart().Z();


  ToolAISShape = new AIS_Shape(ToolTopoShape);
  if (m_context->HasLocation(ToolAISShape)) {
    qDebug() << "m_context->HasLocation(ToolTopoShape)";
  }

  ToolAISShape->SetDisplayMode(AIS_Shaded);
  ToolAISShape->SetColor(Quantity_NOC_BLUE1);
  m_context->Display(ToolAISShape, true);
  m_view->FitAll();


  //设置位姿
  gp_Trsf trans, rot, transs;
  trans.SetTranslationPart(gp_Vec(100, 100, 100));
  gp_Quaternion qua;
  qua.SetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, 0.0, 0.0, 0.0);
  rot.SetRotation(qua);
  transs = trans * rot;
  //m_context->SetLocation(ToolAISShape, transs);
  auto loc = m_context->Location(ToolAISShape);
  qDebug() << "position:" << ToolTopoShape.Location().Transformation().TranslationPart().X() << ","
           << ToolTopoShape.Location().Transformation().TranslationPart().Y() << ","
           << PartTopoShape.Location().Transformation().TranslationPart().Z();
  qDebug() << "position01:" << ToolTopoShape.Location().Transformation().TranslationPart().X() << ","
           << ToolTopoShape.Location().Transformation().TranslationPart().Y() << ","
           << PartTopoShape.Location().Transformation().TranslationPart().Z();
  qDebug() << " loc.Transformation().TranslationPart():" << loc.Transformation().TranslationPart().X() << ","
           << loc.Transformation().TranslationPart().Y() << "," << loc.Transformation().TranslationPart().Z();
  double RX, RY, RZ;
  loc.Transformation().GetRotation().GetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, RX, RY, RZ);
  qDebug() << "loc.Transformation().TranslationPart():" << RX << "," << RY << "," << RZ;

  m_context->Update(ToolAISShape, true);

  m_view->FitAll();
}

void OccView::loadDisplaySTL() {
  //    StlAPI_Reader aReader_Stl;
  //    TopoDS_Shape shape_Stl;
  //    auto res= aReader_Stl.Read(shape_Stl, stlPath.toUtf8());
  //    if(res){
  //        Handle(AIS_Shape) STLAIS=new AIS_Shape(shape_Stl);
  //        m_context->Display(STLAIS,Standard_True);
  //    }

  RWStl reader_stl;
  Handle(Poly_Triangulation) triFace = reader_stl.ReadFile(stlPath.toUtf8());
  Standard_Integer nTriangles = triFace->NbTriangles();
  TColgp_Array1OfPnt nodes(1, triFace->NbNodes());
  Poly_Array1OfTriangle triangles(1, triFace->NbTriangles());
  //  nodes = triFace->Nodes();
  //  triangles = triFace->Triangles();


  //    Handle(AIS_Triangulation) stlais=new AIS_Triangulation(triFace);
  //    //Handle(TColStd_HArray1OfInteger) color=new TColStd_HArray1OfInteger(10);
  //    Handle(TColStd_HArray1OfInteger) anArr = new TColStd_HArray1OfInteger (1, 3);
  //    anArr->SetValue (1,10);
  //    anArr->SetValue (2, 11);
  //    anArr->SetValue (3, 12);
  //    stlais->SetColors(anArr);
  //    m_context->Display(stlais,Standard_True);

  TopoDS_Compound aComp;
  BRep_Builder BuildTool;
  BuildTool.MakeCompound(aComp);

  /*********点呈现**************/
  {
          //        for(Standard_Integer i=nodes.Lower();i<nodes.Upper();i++){
          //            auto point=nodes.Value(i);
          //            TopoDS_Vertex vet=BRepBuilderAPI_MakeVertex(point);

          //            BuildTool.Add( aComp, vet );
          //            qDebug()<<"i:"<<i;

          //        }
          //        Handle(AIS_Shape) ais=new AIS_Shape(aComp);
          //        m_context->Display(ais,Standard_True);
  }


  /*********面片呈现**************/
  {
    //        for(Standard_Integer i=triangles.Lower();i<triangles.Upper();i++){
    //            auto aTriangle=triangles.Value(i);


    //            Standard_Integer nVertexIndex1,nVertexIndex2,nVertexIndex3;
    //            aTriangle.Get(nVertexIndex1, nVertexIndex2, nVertexIndex3);
    //            gp_Pnt  vertex1,vertex2,vertex3;
    //            vertex1 = nodes.Value(nVertexIndex1);
    //            vertex2 = nodes.Value(nVertexIndex2);
    //            vertex3 = nodes.Value(nVertexIndex3);

    //             auto vet=BRepBuilderAPI_MakePolygon(vertex1,vertex2,vertex3);
    //            BuildTool.Add(aComp,vet);
    //            qDebug()<<"i:"<<i;

    //        }
    //        Handle(AIS_Shape) ais=new AIS_Shape(aComp);
    //        m_context->Display(ais,Standard_True);
  }


  qDebug() << "nTriangles:" << nTriangles << "," << triangles.Length();
  m_view->FitAll();
}

void OccView::displayNormalVector() {


  //Building the path curve
  //Building elementaries edges
  TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(0, 1, 0));
  GC_MakeArcOfCircle arc(gp_Pnt(0, 1, 0), gp_Pnt(0.5, 1.5, 0), gp_Pnt(1, 1, 0));
  TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(arc.Value());
  TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(gp_Pnt(1, 1, 0), gp_Pnt(1, 0.5, 0));
  //Joining edges
  TopoDS_Wire spine = BRepBuilderAPI_MakeWire(edge1, edge2, edge3);
  //building profile
  TopoDS_Wire profile = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), 0.1)));
  //Building the pipe
  BRepFill_PipeShell pipeBuilder(spine);
  pipeBuilder.Add(profile);
  pipeBuilder.Build();
  TopoDS_Shape pipeShell = pipeBuilder.Shape();

  //      绘制半圆
  Handle(AIS_Shape) ais_pipeShell = new AIS_Shape(pipeShell);
  m_context->SetColor(ais_pipeShell, Quantity_NOC_GREEN, Standard_True);
  m_context->Display(ais_pipeShell, Standard_True);
  PartAISShape->AddChild(ais_pipeShell);


  //  qDebug() << "displayNormalVector:" ;
  //  //遍历零件的面
  //  for (TopExp_Explorer e(selectFaceShape, TopAbs_FACE); e.More(); e.Next()) {
  //    TopoDS_Face aFace = TopoDS::Face(e.Current());
  //    //拓扑面和几何曲面的方向相反时反转
  //    if (aFace.Orientation() == TopAbs_REVERSED) {
  //      aFace.Reversed();
  //    }
  //    BRepGProp_Face analysisFace(aFace);
  //    Standard_Real umin, umax, vmin, vmax;
  //    analysisFace.Bounds(umin, umax, vmin, vmax);//获取拓扑面的参数范围
  //    Standard_Real midU, midV;
  //    int piece = 10;
  //    midU = (umin + umax) / piece;//拓扑面的参数中点
  //    midV = (vmin + vmax) / piece;
  //    for(auto i=midU;i<piece;++midU){
  //      qDebug()<<"i"<<midU;
  //      for(auto j=midV;j<piece;++midV){
  //        qDebug()<<"j"<<midV;
  //        gp_Vec norm;
  //        gp_Pnt normPoint;
  //        analysisFace.Normal(i, j, normPoint, norm);//返回面参数中点的坐标及其法向
  //        //绘制法线
  //        gp_Lin normLine(normPoint, gp_Dir(norm));
  //        TopoDS_Edge anEdge = BRepBuilderAPI_MakeEdge(normLine, 0, 20);
  //        NormalVector.push_back(new AIS_Shape(anEdge));
  //        m_context->Display(NormalVector.back(), Standard_True);
  //      }
  //    }
  //
  //
  //    gp_Vec norm;
  //    gp_Pnt midPoint;
  //    analysisFace.Normal(midU, midV, midPoint, norm);//返回面参数中点的坐标及其法向
  //    //绘制法线
  //    gp_Lin normLine(midPoint, gp_Dir(norm));
  //    TopoDS_Edge anEdge = BRepBuilderAPI_MakeEdge(normLine, 0, 20);
  //    NormalVector.push_back(new AIS_Shape(anEdge));
  //    m_context->Display(NormalVector.back(), Standard_True);


  //Handle(AIS_Shape) ais_shape = new AIS_Shape(anEdge);
  //m_context->Display(ais_shape, Standard_True);
  //  }
}

void OccView::removeNormalVector() {
  if (NormalVector.back()) {
    m_context->Remove(NormalVector.back(), Standard_True);
  } else {
    qDebug() << "no vector";
  }
}

void OccView::InitView() {
  if (m_context.IsNull()) {
    //此对象提供与X server的连接，在Windows和Mac OS中不起作用
    Handle(Aspect_DisplayConnection) m_display_donnection = new Aspect_DisplayConnection();
    //创建OpenGl图形驱动
    if (m_graphic_driver.IsNull()) {
      m_graphic_driver = new OpenGl_GraphicDriver(m_display_donnection);
    }
    //获取QWidget的窗口系统标识符
    WId window_handle = (WId) winId();
    //创建Windows NT 窗口
    Handle(WNT_Window) wind = new WNT_Window((Aspect_Handle) window_handle);
    //创建3D查看器
    m_viewer = new V3d_Viewer(m_graphic_driver /*, Standard_ExtString("viewer3d")*/);
    //创建视图
    m_view = m_viewer->CreateView();
    m_view->SetWindow(wind);
    //打开窗口
    if (!wind->IsMapped()) {
      wind->Map();
    }
    m_context = new AIS_InteractiveContext(m_viewer);//创建交互式上下文
    //配置查看器的光照
    m_viewer->SetDefaultLights();
    m_viewer->SetLightOn();
    //设置视图的背景颜色为灰色
    m_view->SetBackgroundColor(Quantity_NOC_GRAY60);
    m_view->MustBeResized();
    //显示直角坐标系，可以配置在窗口显示位置、文字颜色、大小、样式
    // m_view->TriedronDisplay(Aspect_TOTP_CENTER, Quantity_NOC_GOLD, 0.08, V3d_WIREFRAME);
    //设置显示模式
    m_context->SetDisplayMode(AIS_Shaded, Standard_True);


    opencascade::handle<AIS_ViewCube> aisViewCube = new AIS_ViewCube;
    aisViewCube->SetBoxColor(Quantity_NOC_GRAY75);
    aisViewCube->SetFixedAnimationLoop(false);
    aisViewCube->SetSize(55);
    aisViewCube->SetFontHeight(12);
    aisViewCube->SetAxesLabels("", "", "");
    aisViewCube->SetTransformPersistence(
            new Graphic3d_TransformPers(
                    Graphic3d_TMF_TriedronPers,
                    Aspect_TOTP_LEFT_LOWER,
                    Graphic3d_Vec2i(85, 85)));

    //    aisViewCube->Attributes()->DatumAspect()->LineAspect(Prs3d_DP_XAxis)->SetColor(Quantity_NOC_RED2);
    const Handle_Prs3d_DatumAspect &datumAspect = aisViewCube->Attributes()->DatumAspect();
    //    datumAspect->ShadingAspect(Prs3d_DP_XAxis)->SetColor(Quantity_NOC_RED2);
    //    datumAspect->ShadingAspect(Prs3d_DP_YAxis)->SetColor(Quantity_NOC_GREEN2);
    //    datumAspect->ShadingAspect(Prs3d_DP_ZAxis)->SetColor(Quantity_NOC_BLUE2);
    m_context->Display(aisViewCube, true);


    //获取原始坐标系
    Handle_Geom_Axis2Placement axis = new Geom_Axis2Placement(gp::XOY());
    Handle_AIS_Trihedron aisTrihedron = new AIS_Trihedron(axis);
    aisTrihedron->SetDatumDisplayMode(Prs3d_DM_WireFrame);
    aisTrihedron->SetDrawArrows(false);
    aisTrihedron->Attributes()->DatumAspect()->LineAspect(Prs3d_DP_XAxis)->SetWidth(2.5);
    aisTrihedron->Attributes()->DatumAspect()->LineAspect(Prs3d_DP_YAxis)->SetWidth(2.5);
    aisTrihedron->Attributes()->DatumAspect()->LineAspect(Prs3d_DP_ZAxis)->SetWidth(2.5);
    aisTrihedron->SetDatumPartColor(Prs3d_DP_XAxis, Quantity_NOC_RED2);
    aisTrihedron->SetDatumPartColor(Prs3d_DP_YAxis, Quantity_NOC_GREEN2);
    aisTrihedron->SetDatumPartColor(Prs3d_DP_ZAxis, Quantity_NOC_BLUE2);
    aisTrihedron->SetLabel(Prs3d_DP_XAxis, "X");
    aisTrihedron->SetLabel(Prs3d_DP_YAxis, "Y");
    aisTrihedron->SetLabel(Prs3d_DP_ZAxis, "Z");
    aisTrihedron->SetTextColor(Quantity_NOC_GRAY40);
    aisTrihedron->SetSize(60);
    aisTrihedron->SetTransformPersistence(
            new Graphic3d_TransformPers(Graphic3d_TMF_ZoomPers, axis->Ax2().Location()));
    aisTrihedron->Attributes()->SetZLayer(Graphic3d_ZLayerId_Topmost);
    aisTrihedron->SetInfiniteState(true);
    m_context->Display(aisTrihedron, true);

    //        for (TopExp_Explorer exp (theBox,TopAbs_FACE);exp.More();exp.Next()) {
    //            TopoDS_Face aCurrentFace = TopoDS::Face(exp.Current());
    //            //测试当前面的方向
    //            TopAbs_Orientation orient = aCurrentFace.Orientation();
    //            //重新生成几何平面
    //            TopLoc_Location location;
    //            Handle (Geom_Surface) aGeometricSurface = BRep_Tool::Surface(aCurrentFace,location);
    //            Handle (Geom_Plane) aPlane = Handle (Geom_Plane)::DownCast(aGeometricSurface);
    //            Handle(AIS_Shape) Blue01 =new AIS_Shape(aPlane);
    //            m_context->Display(aPlane,Standard_True);
    //            //Build an AIS_Shape with a new color
    //            //创建一个新的AIS_Shape
    //            Handle(AIS_Shape) theMovingFace = new AIS_Shape(aCurrentFace);
    //            Quantity_NameOfColor aCurrentColor = (Quantity_NameOfColor)j;
    //            m_context->SetColor(theMovingFace,aCurrentColor,Standard_False);
    //            m_context->SetMaterial(theMovingFace,Graphic3d_NOM_PLASTIC,Standard_False);

    //            //查找每个面的法向量
    //            gp_Pln agpPlane = aPlane->Pln();


    //            gp_Ax1 norm = agpPlane.Axis();
    //            gp_Dir dir = norm.Direction();
    //            gp_Vec move(dir);
    //            TopLoc_Location aLocation;
    //            Handle (AIS_ConnectedInteractive) theTransformedDisplay = new AIS_ConnectedInteractive();
    //            theTransformedDisplay->Connect(theMovingFace, aLocation);
    //            // = myAISContext->Location(theMovingFace);
    //            Handle (Geom_Transformation) theMove = new Geom_Transformation(aLocation.Transformation());
    //            for (Standard_Integer i=1;i<=30;i++) {
    //                theMove->SetTranslation(move*i);
    //                if (orient==TopAbs_FORWARD) m_context->SetLocation(theTransformedDisplay,TopLoc_Location(theMove->Trsf()));
    //               else m_context->SetLocation(theTransformedDisplay,TopLoc_Location(theMove->Inverted()->Trsf()));
    //                m_context->Redisplay(theTransformedDisplay,Standard_False);
    //            }

    //            j+=15;
    //        }


    //        TopoDS_Vertex V1 = BRepBuilderAPI_MakeVertex(gp_Pnt(-200.,-200.,0.));

    //        gp_Trsf theTransformation;
    //        gp_Ax1 axe(gp_Pnt(110, 60, 60),  gp_Dir(0.0, 1.0, 0.0));
    //        theTransformation.SetMirror(axe);
    //        BRepBuilderAPI_Transform myBRepTransformation(V1,theTransformation,false);
    //        TopoDS_Shape S2 = myBRepTransformation.Shape();
    //        Handle(AIS_Shape) ddd=new AIS_Shape(V1);
    //        m_context->Display(ddd,true);
    //        Handle(AIS_Shape) ddd01=new AIS_Shape(S2);
    //        m_context->Display(ddd01,true);

    //        TopoDS_Edge E11 = BRepBuilderAPI_MakeEdge(gp_Pnt(40.,0.,0.), gp_Pnt(82.5,25.,0.));
    //        TopoDS_Edge E15 = BRepBuilderAPI_MakeEdge(gp_Pnt(82.5,25.,0.), gp_Pnt(42.5,93.,0.));
    //        TopoDS_Edge E12 = BRepBuilderAPI_MakeEdge(gp_Pnt(82.5,25.,0.), gp_Pnt(42.5,93.,0.));
    //        TopoDS_Edge E13 = BRepBuilderAPI_MakeEdge(gp_Pnt(42.5,93.,0.), gp_Pnt(0.,68.,0.));
    //        TopoDS_Edge E14 = BRepBuilderAPI_MakeEdge(gp_Pnt(0.,68.,0.), gp_Pnt(40.,0.,0.));
    //        TopoDS_Wire W1 = BRepBuilderAPI_MakeWire(E11,E12,E13,E14);
    //        Handle(AIS_Shape) ddd02=new AIS_Shape(W1);
    //        m_context->Display(ddd02,Standard_True);


    //        gp_Pln aPlane;

    //        gp_Circ aCircle1(gp::XOY(), 1.0);
    //        gp_Circ aCircle2(gp::XOY(), 1.0);
    //        gp_Circ aCircle3(gp::XOY(), 1.0);

    //        Handle(AIS_Shape) ddd03=new AIS_Shape(BRepBuilderAPI_MakeEdge(aCircle3));

    //        aCircle1.SetLocation(gp_Pnt(3.0, 3.0, 0.0));
    //        aCircle2.SetLocation(gp_Pnt(7.0, 3.0, 0.0));
    //        aCircle3.SetLocation(gp_Pnt(3.0, 7.0, 0.0));

    //        BRepBuilderAPI_MakeEdge anEdgeMaker1(aCircle1);
    //        BRepBuilderAPI_MakeEdge anEdgeMaker2(aCircle2);
    //        BRepBuilderAPI_MakeEdge anEdgeMaker3(aCircle3);

    //        BRepBuilderAPI_MakeWire aWireMaker1(anEdgeMaker1.Edge());
    //        BRepBuilderAPI_MakeWire aWireMaker2(anEdgeMaker2.Edge());
    //        BRepBuilderAPI_MakeWire aWireMaker3(anEdgeMaker3.Edge());

    //        BRepBuilderAPI_MakeFace aFaceMaker(aPlane, 0.0, 10.0, 0.0, 10.0);

    //        Geom_CartesianPoint p01(gp_Pnt(0,0,0));
    //        Geom_Plane pln1(aPlane);


    //        gp_Ax2 axis00(gp_Pnt(1, 2, 3), gp::DZ());
    //        std::ofstream dumpFile("geometrySurface.txt");
    //        Handle_Geom_Circle baseCurve = new Geom_Circle(axis00, 4.0);
    //        Handle_Geom_SurfaceOfLinearExtrusion theExtrusion = new Geom_SurfaceOfLinearExtrusion(baseCurve, gp_Dir(0, 0.6, 0.8));
    //        //        GeomTools::Write(theExtrusion, dumpFile);
    //        //        GeomTools::Dump(theExtrusion, dumpFile);
    //        //        GeomTools::Write(theExtrusion, std::cout);
    //        //        GeomTools::Dump(theExtrusion, std::cout);

    //        Handle_Geom_CylindricalSurface theCylinder = new Geom_CylindricalSurface(axis00, 4.0);
    //        TopoDS_Shape cylin=BRepPrimAPI_MakeCylinder(3,20,10);
    //        Handle(AIS_Shape) aisCylin=new AIS_Shape(cylin);
    //        m_context->Display(aisCylin,Standard_True);
  }
  //配置QWidget
  setAttribute(Qt::WA_PaintOnScreen);
  setAttribute(Qt::WA_NoSystemBackground);
  setBackgroundRole(QPalette::NoRole);//无背景
  setFocusPolicy(Qt::StrongFocus);
  setAttribute(Qt::WA_PaintOnScreen);
  setAttribute(Qt::WA_NoSystemBackground);
  setMouseTracking(true);//开启鼠标位置追踪
}

void OccView::InitFilters() {
  aFil1 = new StdSelect_FaceFilter(StdSelect_Revol);
  aFil2 = new StdSelect_FaceFilter(StdSelect_Plane);
}

void OccView::pickUp(TopoDS_Shape) {
  //提取工件中所有的面
  TopExp_Explorer Ex;
  workpiece_show_faces.clear();
  int index = 0;
  for (Ex.Init(PartTopoShape, TopAbs_FACE); Ex.More(); Ex.Next()) {
    index++;                                              //按顺序给定面的索引
    TopoDS_Face current_face = TopoDS::Face(Ex.Current());//将资源管理器里面的面对象转到容器中
    Show_face result;
    result.face = current_face;
    result.adv_face_index = index;
    workpiece_show_faces.push_back(result);//这边就是将result这个结构体放入容器workpiece_show_faces中
  }
}

void OccView::getShape() {
  if (m_context->HasDetectedShape()) {
    TopoDS_Shape abc = m_context->DetectedShape();
    selectFaceShape = abc;
    TopAbs_ShapeEnum bba = abc.ShapeType();
    qDebug() << bba;

    int size = workpiece_show_faces.size();
    for (int i = 0; i < size; i++) {
      TopoDS_Shape cur_face = workpiece_show_faces.at(i).face;
      Standard_Boolean abd = cur_face.IsEqual(abc);
      if (abd) {
        qDebug() << workpiece_show_faces.at(i).adv_face_index;
      }
    }
  }
}

void OccView::selectMode(Handle(AIS_Shape) selectmode, const Ui::selectionType &type) {
  //shading visualization mode, no specific mode, authorization for decomposition into sub-shapes
  //着色可视化模式，无特定模式，授权分解为子形状
  //const TopoDS_Shape theShape;
  //Handle(AIS_Shape) aShapePrs = new AIS_Shape(theShape);
  //m_context->Display(aShapePrs, AIS_Shaded, -1, true, true);
  //m_context->Display(selectmode, AIS_Shaded, -1, true, true);
  //activates decomposition of shapes into faces
  //激活将形状分解为面
  //auto aSubShapeSelMode = AIS_Shape::SelectionMode(TopAbs_VERTEX);
  //m_context->Activate(aShapePrs, aSubShapeSelMode);
  //m_context->Activate(selectmode, aSubShapeSelMode);
  //m_context->AddFilter(aFil1);
  //m_context->AddFilter(aFil2);
  //only faces of revolution or planar faces will be selected
  //仅选择旋转面或平面
  //m_context->MoveTo(thePixelX, thePixelY, myView, true);

  //m_context->RemoveFilter(aFil1);
  //m_context->RemoveFilter(aFil2);

  m_context->Display(selectmode, AIS_Shaded, -1, true, true);

  if (type == Ui::selectionType::FaceSel) {
    auto aSubShapeSelMode = AIS_Shape::SelectionMode(TopAbs_FACE);
    m_context->Activate(selectmode, aSubShapeSelMode);
  } else if (type == Ui::selectionType::VetextSel) {
    auto aSubShapeSelMode = AIS_Shape::SelectionMode(TopAbs_VERTEX);
    m_context->Activate(selectmode, aSubShapeSelMode);
  } else if (type == Ui::selectionType::EdgeSel) {
    auto aSubShapeSelMode = AIS_Shape::SelectionMode(TopAbs_EDGE);
    m_context->Activate(selectmode, aSubShapeSelMode);
  }
}

void OccView::DisselectMode(Handle(AIS_Shape) selectmode, const Ui::selectionType &type) {
  if (type == Ui::selectionType::FaceSel) {
    auto aSubShapeSelMode = AIS_Shape::SelectionMode(TopAbs_FACE);
    m_context->Deactivate(selectmode, aSubShapeSelMode);
  } else if (type == Ui::selectionType::VetextSel) {
    auto aSubShapeSelMode = AIS_Shape::SelectionMode(TopAbs_VERTEX);
    m_context->Deactivate(selectmode, aSubShapeSelMode);
  } else if (type == Ui::selectionType::EdgeSel) {
    auto aSubShapeSelMode = AIS_Shape::SelectionMode(TopAbs_EDGE);
    m_context->Deactivate(selectmode, aSubShapeSelMode);
  }
}

void OccView::newPartCoordinate() {
  //    gp_Pnt oPnt(0, 0, 0);//原点

  //    gp_Pnt xPnt(1, 1, 1);
  //    gp_Vec vX(oPnt, xPnt);
  //    gp_Dir aixX(vX / vX.Magnitude());//x方向

  //    gp_Vec constructV= vX.Rotated(gp_Ax1(oPnt,gp_Dir(0,0,1)),1.571);	//对X轴进行旋转构造一个向量叉乘计算Z轴
  //    gp_Vec vZ = vX.Crossed(constructV);
  //    gp_Dir aixZ = (vZ / vZ.Magnitude());//z方向

  //    gp_Ax2 ax = gp_Ax2(oPnt, aixZ, aixX);
  //    Handle(Geom_Axis2Placement) TrihedronAxis = new Geom_Axis2Placement(ax);
  //    Handle(AIS_Trihedron) partTrihedron = new AIS_Trihedron(TrihedronAxis);
  //    m_context->Display(partTrihedron, Standard_True);

  //    gp_Ax3 ay0(ax);
  //    ay0.XReverse();

  //    ay0.Ax2();


  if (firstPointSelected && secondPointSelected && thirdPointSelected) {
    firstPointSelected = secondPointSelected = thirdPointSelected = false;
  }

  if (m_context->HasDetectedShape()) {
    TopoDS_Shape abc = m_context->DetectedShape();
    if (abc.ShapeType() == TopAbs_ShapeEnum::TopAbs_VERTEX) {
      if (!firstPointSelected) {
        TopoDS_Vertex V = TopoDS::Vertex(abc);
        pointO = BRep_Tool::Pnt(V);
        firstPointSelected = true;
        qDebug() << pointO.X() << "," << pointO.Y() << "," << pointO.Z();
        gp_Pnt gPnt(100.0, 100.0, 100.0);


      } else if (firstPointSelected && !secondPointSelected) {
        TopoDS_Vertex V = TopoDS::Vertex(abc);
        pointX = BRep_Tool::Pnt(V);
        secondPointSelected = true;
        qDebug() << pointX.X() << "," << pointX.Y() << "," << pointX.Z();
      } else if (firstPointSelected && secondPointSelected && !thirdPointSelected) {
        TopoDS_Vertex V = TopoDS::Vertex(abc);
        pointY = BRep_Tool::Pnt(V);
        thirdPointSelected = true;
        qDebug() << pointY.X() << "," << pointY.Y() << "," << pointY.Z();
        gp_Vec vX(pointO, pointX);
        gp_Dir aixX(vX / vX.Magnitude());//x方向

        gp_Vec vY(pointO, pointY);
        gp_Dir aixY(vY / vY.Magnitude());//y方向

        gp_Vec vZ = vX.Crossed(vY);

        gp_Dir aixZ = (vZ / vZ.Magnitude());//z方向

        part0Ax2 = gp_Ax2(pointO, aixZ, aixX);
        PartTrihedronAxis = new Geom_Axis2Placement(part0Ax2);
        partTrihedron = new AIS_Trihedron(PartTrihedronAxis);

        m_context->Display(partTrihedron, Standard_True);
        partTrihedron->AddChild(PartAISShape);
        RobotAISShape[6]->AddChild(partTrihedron);


        gp_Ax3 origin(gp::XOY());
        gp_Ax3 current(part0Ax2);
        gp_Trsf tran;
        tran.SetTransformation(current, origin);
        auto posTran = tran.TranslationPart();
        qDebug() << "NewCoordinatePostion:" << posTran.X() << "," << posTran.Y() << "," << posTran.Z();
        double Rx, Ry, Rz;
        tran.GetRotation().GetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, Rx, Ry, Rz);
        qDebug() << "NewCoordinateVelity:" << Rx << "," << Ry << "," << Rz;

        getPartCoor().x = originPart0Coordinate.x = posTran.X();
        getPartCoor().y = originPart0Coordinate.y = posTran.Y();
        getPartCoor().z = originPart0Coordinate.z = posTran.Z();
        getPartCoor().rx = originPart0Coordinate.rx = Rx;
        getPartCoor().ry = originPart0Coordinate.ry = Ry;
        getPartCoor().rz = originPart0Coordinate.rz = Rz;

        emit NewPartCoordinateCompleteSigal();
      }
    }
  }

  gp_GTrsf trsf01;


  //    Handle(Geom_Axis2Placement) TrihedronAxisL = new Geom_Axis2Placement(ax);
  //    gp_Dir yDirection = TrihedronAxisL->YDirection();
  //    gp_Pnt location = TrihedronAxisL->Ax2().Location();
  //    gp_Ax1 retation_axiY(location, yDirection);
  //    gp_Trsf rotation_trans;
  //    rotation_trans.SetRotation(retation_axiY, deg);
  //    TrihedronAxisL->Transform(rotation_trans);
  //    Handle(AIS_Trihedron) partTrihedron = new AIS_Trihedron(TrihedronAxisL);
  //    m_context->Display(partTrihedron, Standard_True);
  //    m_view->FitAll();
  //    return;
}

void OccView::newToolCoordinate() {
  if (m_context->HasDetectedShape()) {
    TopoDS_Shape abc = m_context->DetectedShape();
    if (abc.ShapeType() == TopAbs_ShapeEnum::TopAbs_VERTEX) {
      if (!firstPointSelected) {
        TopoDS_Vertex V = TopoDS::Vertex(abc);
        pointO = BRep_Tool::Pnt(V);
        firstPointSelected = true;
        qDebug() << "pointO:" << pointO.X() << "," << pointO.Y() << "," << pointO.Z();
        gp_Pnt gPnt(100.0, 100.0, 100.0);

        /*Standard_CString str = "first point:";
        TCollection_ExtendedString extStr(str);

        Handle(AIS_TextLabel) aPntLabel = new AIS_TextLabel();
        aPntLabel->SetText(extStr);
        aPntLabel->SetPosition(gPnt);
        aPntLabel->SetFont("Microsoft yahei");
        m_context->Display(aPntLabel, Standard_True);*/

      } else if (firstPointSelected && !secondPointSelected) {
        TopoDS_Vertex V = TopoDS::Vertex(abc);
        pointX = BRep_Tool::Pnt(V);
        secondPointSelected = true;
        qDebug() << pointX.X() << "," << pointX.Y() << "," << pointX.Z();
      } else if (firstPointSelected && secondPointSelected && !thirdPointSelected) {
        TopoDS_Vertex V = TopoDS::Vertex(abc);
        pointY = BRep_Tool::Pnt(V);
        thirdPointSelected = true;
        qDebug() << pointY.X() << "," << pointY.Y() << "," << pointY.Z();
        gp_Vec vX(pointO, pointX);
        gp_Dir aixX(vX / vX.Magnitude());//x方向

        gp_Vec vY(pointO, pointY);
        gp_Dir aixY(vY / vY.Magnitude());//y方向

        //gp_Vec constructV = vX.Rotated(gp_Ax1(pointO, vY), -1.571);	//对X轴进行旋转构造一个向量叉乘计算Z轴
        gp_Vec vZ = vX.Crossed(vY);

        //gp_Dir aixZ = (vZ / vZ.Magnitude());//z方向
        gp_Dir aixZ = (vZ / vZ.Magnitude());//z方向

        part0Ax2 = gp_Ax2(pointO, aixZ, aixX);

        ToolTrihedronAxis = new Geom_Axis2Placement(part0Ax2);

        toolTrihedron = new AIS_Trihedron(ToolTrihedronAxis);

        gp_Vec vY_new = vZ.Crossed(vX);
        gp_Dir aixY_new = vY_new / vY_new.Magnitude();

        /*		Eigen::AngleAxisd rotation_vector(alpha, Eigen::Vector3d(x, y, z));
            Eigen::Vector3d ttt();*/


        m_context->Display(toolTrihedron, Standard_True);
        toolTrihedron->AddChild(ToolAISShape);
        Handle(Geom_Axis2Placement) originAxis = new Geom_Axis2Placement(gp::XOY());
        Handle(AIS_Trihedron) originTrihedron = new AIS_Trihedron(originAxis);
        originTrihedron->SetDatumPartColor(Prs3d_DP_XAxis, Quantity_NOC_RED2);
        originTrihedron->SetDatumPartColor(Prs3d_DP_YAxis, Quantity_NOC_GREEN2);
        originTrihedron->SetDatumPartColor(Prs3d_DP_ZAxis, Quantity_NOC_BLUE2);
        m_context->Display(originTrihedron, Standard_True);
        toolTrihedron->AddChild(originTrihedron);


        gp_Ax3 origin(gp::XOY());
        gp_Ax3 current(part0Ax2);

        gp_Trsf toolTrihedronOriginTran;

        toolTrihedronOriginTran.SetTransformation(current, origin);


        auto transPM = Ui::transf2Matrix_02(toolTrihedronOriginTran);

        double pe[6]{0.0};
        /*	pe[0] = pointO.X();   pe[1] = pointO.Y();   pe[2] = pointO.Z();
          if (aixZ.X() != 1) {
            pe[4] = asin(aixZ.X());
            pe[3] = -asin(aixZ.Y() / cos(pe[4]));
            pe[5] = asin(aixX.Y()*cos(pe[3]) + aixX.Z()*sin(pe[3]));
          }
          else {
            pe[4] = PI / 2;
            pe[3] = PI / 2;
            pe[5] = 0.0;
          }*/

        Ui::s_pm2pe(transPM, pe, "123");

        qDebug() << "result::::";
        qDebug() << pe[0];
        qDebug() << pe[1];
        qDebug() << pe[2];
        qDebug() << pe[3] * 180 / PI_OCC;
        qDebug() << pe[4] * 180 / PI_OCC;
        qDebug() << pe[5] * 180 / PI_OCC;

        qDebug() << "aixX:" << aixX.X() << "," << aixX.Y() << "," << aixX.Z();
        qDebug() << "aixY_new:" << aixY_new.X() << "," << aixY_new.Y() << "," << aixY_new.Z();
        qDebug() << "aixZ:" << aixZ.X() << "," << aixZ.Y() << "," << aixZ.Z();

        getToolCoor().x = originTool0Coordinate.x = pe[0];
        getToolCoor().y = originTool0Coordinate.y = pe[1];
        getToolCoor().z = originTool0Coordinate.z = pe[2];
        getToolCoor().rx = originTool0Coordinate.rx = pe[3];
        getToolCoor().ry = originTool0Coordinate.ry = pe[4];
        getToolCoor().rz = originTool0Coordinate.rz = pe[5];

        //Eigen::Quaterniond quaternion(1, 2, 3, 4);
        //Eigen::Matrix4d matrixxx;
        //

        //auto posTran = toolTrihedronOriginTran.TranslationPart();
        //qDebug() << "NewCoordinatePostion:" << posTran.X() << "," << posTran.Y() << "," << posTran.Z();
        //double Rx, Ry, Rz;
        //toolTrihedronOriginTran.GetRotation().GetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, Rx, Ry, Rz);
        //qDebug() << "NewCoordinateVelity:" << Rx << "," << Ry << "," << Rz;

        ////getToolCoor().x = originTool0Coordinate.x = posTran.X(); getToolCoor().y = originTool0Coordinate.y = posTran.Y(); getToolCoor().z = originTool0Coordinate.z = posTran.Z();
        ////getToolCoor().rx = originTool0Coordinate.rx = Rx; getToolCoor().ry = originTool0Coordinate.ry = Ry; getToolCoor().rz = originTool0Coordinate.rz = Rz;


        //auto loc = m_context->Location(toolTrihedron);

        //loc.Transformation().GetRotation().GetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, Rx, Ry, Rz);
        //qDebug() << loc.Transformation().TranslationPart().X() << loc.Transformation().TranslationPart().Y() << loc.Transformation().TranslationPart().Z();
        //qDebug() << Rx; qDebug() << Ry; qDebug() << Rz;

        m_context->Update(toolTrihedron, true);
        m_context->Update(originTrihedron, true);

        emit NewToolCoordinateCompleteSigal();
      }
    }
  }
}


void OccView::coutPointCoor() {
  if (m_context->HasDetectedShape()) {
    TopoDS_Shape abc = m_context->DetectedShape();
    if (abc.ShapeType() == TopAbs_ShapeEnum::TopAbs_VERTEX) {
      TopoDS_Vertex V = TopoDS::Vertex(abc);
      pointO = BRep_Tool::Pnt(V);
      qDebug() << "PointCoordinate:" << pointO.X() << "," << pointO.Y() << "," << pointO.Z();
    }
  }
}

void OCAFBrowser::load(QTreeWidget *theTree) {
  theTree->clear();

  QTreeWidgetItem *root = new QTreeWidgetItem();
  root->setText(0, QLatin1String("0"));
  root->setIcon(0, myGroupIcon);
  theTree->addTopLevelItem(root);
  load(pDoc->GetData()->Root(), root, QString::fromLatin1("0"));

  //    {
  //        TDF_Label mainLabel = pDoc->Main();
  //        Handle(XCAFDoc_ShapeTool) ShapeTool = XCAFDoc_DocumentTool::ShapeTool(mainLabel);
  //        Handle(XCAFDoc_ColorTool) ColorTool = XCAFDoc_DocumentTool::ColorTool(mainLabel);
  //        TDF_LabelSequence tdfLabels;
  //        ShapeTool->GetFreeShapes(tdfLabels);   //获取装配体和组件对应名称
  //        int Roots = tdfLabels.Length();
  //        TDF_Label Label = tdfLabels.Value(1);
  //        TDF_LabelSequence components;
  //        ShapeTool->GetComponents(Label, components);
  //        qDebug()<<"components:"<<components.Length();
  //        for (Standard_Integer compIndex = 1; compIndex <= components.Length(); ++compIndex)
  //        {
  //            TDF_Label ChildLabel = components.Value(compIndex);
  //            Handle(TDataStd_Name) name;
  //            if (ChildLabel.FindAttribute(TDataStd_Name::GetID(), name)) {
  //                QTreeWidgetItem *item=new QTreeWidgetItem();
  //                QString text = QString::fromLatin1("%1").arg(QString::fromUtf8(toString(name->Get()).c_str()));
  //                item->setText(0, text);
  //                root->addChild(item);
  //            }

  //        }

  //        TDF_Label Label00 = components.Value(2);

  //        Handle(XCAFDoc_ShapeTool) ShapeTool00 = XCAFDoc_DocumentTool::ShapeTool(mainLabel);

  //        auto shape=ShapeTool00->GetShape(Label00);
  //        gp_Vec gv(100,200,300);
  //        gp_Trsf tf;
  //        tf.SetTranslationPart(gv);


  //        TopLoc_Location loctf(tf);
  //        shape.Move(loctf);

  //        auto res=ShapeTool00->IsAssembly(Label00);
  //        qDebug()<<"isShape:"<<res;


  //    }
}

void OCAFBrowser::load(const TDF_Label &label, QTreeWidgetItem *item, const QString &s) {

  Handle(TDataStd_Name) name;
  if (label.FindAttribute(TDataStd_Name::GetID(), name)) {
    QString text = QString::fromLatin1("%1 %2").arg(s).arg(QString::fromUtf8(toString(name->Get()).c_str()));
    item->setText(0, text);
  }

  for (TDF_ListIteratorOfIDList it(myList); it.More(); it.Next()) {
    Handle(TDF_Attribute) attr;
    if (label.FindAttribute(it.Value(), attr)) {
      QTreeWidgetItem *child = new QTreeWidgetItem();
      if (it.Value() == TDataStd_Name::GetID()) {
        QString text;
        QTextStream str(&text);
        str << attr->DynamicType()->Name();
        str << " = " << toString(Handle(TDataStd_Name)::DownCast(attr)->Get()).c_str();
        child->setText(0, text);
        item->addChild(child);
      } else if (it.Value() == TDF_TagSource::GetID()) {
        QString text;
        QTextStream str(&text);
        str << attr->DynamicType()->Name();
        str << " = " << Handle(TDF_TagSource)::DownCast(attr)->Get();
        child->setText(0, text);
        item->addChild(child);
      } else if (it.Value() == TDataStd_Integer::GetID()) {
        QString text;
        QTextStream str(&text);
        str << attr->DynamicType()->Name();
        str << " = " << Handle(TDataStd_Integer)::DownCast(attr)->Get();
        child->setText(0, text);
        item->addChild(child);
      } else if (it.Value() == TNaming_NamedShape::GetID()) {
        TopoDS_Shape shape = Handle(TNaming_NamedShape)::DownCast(attr)->Get();
        QString text;
        QTextStream str(&text);
        str << attr->DynamicType()->Name() << " = ";
        if (!shape.IsNull()) {
          switch (shape.ShapeType()) {
            case TopAbs_COMPOUND:
              str << "COMPOUND PRIMITIVE";
              break;
            case TopAbs_COMPSOLID:
              str << "COMPSOLID PRIMITIVE";
              break;
            case TopAbs_SOLID:
              str << "SOLID PRIMITIVE";
              break;
            case TopAbs_SHELL:
              str << "SHELL PRIMITIVE";
              break;
            case TopAbs_FACE:
              str << "FACE PRIMITIVE";
              break;
            case TopAbs_WIRE:
              str << "WIRE PRIMITIVE";
              break;
            case TopAbs_EDGE:
              str << "EDGE PRIMITIVE";
              break;
            case TopAbs_VERTEX:
              str << "VERTEX PRIMITIVE";
              break;
            case TopAbs_SHAPE:
              str << "SHAPE PRIMITIVE";
              break;
          }
        }
        child->setText(0, text);
        item->addChild(child);
      } else {
        child->setText(0, QLatin1String(attr->DynamicType()->Name()));
        item->addChild(child);
      }
    }
  }

  int i = 1;
  for (TDF_ChildIterator it(label); it.More(); it.Next(), i++) {
    QString text = QString::fromLatin1("%1:%2").arg(s).arg(i);
    QTreeWidgetItem *child = new QTreeWidgetItem();
    child->setText(0, text);
    child->setIcon(0, myGroupIcon);
    item->addChild(child);
    load(it.Value(), child, text);
  }
}

gp_Pnt OccView::ChangeCoordinateSecond(double x, double y) {
  gp_Pnt P0, P1, P3;
  gp_Vec Vp2;
  //Handle(Geom_Curve) aCurve;
  double X, Y, Z, VX, VY, VZ;
  this->getView()->Convert(int(x), int(y), X, Y, Z);
  this->getView()->Proj(VX, VY, VZ);
  P1.SetCoord(X, Y, Z);
  Vp2.SetCoord(VX, VY, VZ);
  gp_Lin gpLin(P1, gp_Dir(Vp2));

  //
  //   auto aCurve=GC_MakeSegment(gpLin,-10000,10000);
  //   //Handle(Geom_Surface) aGeometricSurface;

  //    GeomAPI_IntCS CS (aCurve,aGeometricSurface);   //aGeometricSurface为与直线相交的面

  //    Standard_Integer NbSeg = 0;
  //    Standard_Integer NbPoints = 0;
  //    Handle(Geom_Curve) segment;
  //    if(CS.IsDone())
  //    {
  //        NbSeg = CS.NbSegments();
  //        for (Standard_Integer k=1; k<=NbSeg; k++)
  //        {
  //            segment = CS.Segment(k);
  //        }

  //        NbPoints = CS.NbPoints();
  //        for (int k=1; k<=NbPoints; k++)
  //        {
  //            gp_Pnt aPoint=CS.Point(k);
  //            P3=aPoint;
  //        }
  //    }
  return P3;
}

void OccView::AnaminationStart() {
  //    qDebug()<<"start";
  //    gp_Trsf endRotation;
  //    gp_Quaternion endaQ;
  //    endaQ.SetEulerAngles(gp_YawPitchRoll,0.12,0,0);
  //    endRotation.SetRotation(endaQ);

  //    //设置平移向量
  //    gp_Trsf endranslation;
  //    gp_Vec theVectorOfTransEnd(10,0,0);
  //    endranslation.SetTranslation(theVectorOfTransEnd);
  //    gp_Trsf endTrsf = endranslation * endRotation;


  //    gp_Trsf startRotation;
  //    gp_Quaternion startaQ;
  //    startaQ.SetEulerAngles(gp_YawPitchRoll,0,0,0);
  //    endRotation.SetRotation(startaQ);

  //    //设置平移向量
  //    gp_Trsf startranslation;
  //    gp_Vec theVectorOfTransStart(0,0,0);
  //    endranslation.SetTranslation(theVectorOfTransStart);
  //    gp_Trsf startTrsf= startranslation * startRotation;


  //    m_context->SetLocation(getAISShape(),endTrsf);
  //    m_context->Update(getAISShape(),Standard_True);


  //    int currentTime=0;
  //    QEasingCurve m_easingCurve;
  //    for (int k=0;k<1000;k++) {
  //        currentTime++;
  //        qDebug()<<currentTime;
  //        double t = m_easingCurve.valueForProgress(currentTime / double(1000));
  //        qDebug()<<t;
  //        const bool prevImmediateUpdate = m_view->SetImmediateUpdate(false);
  //        qDebug()<<"1";
  //        NCollection_Lerp<gp_Vec>  TransLerp;

  //        NCollection_Lerp<gp_Quaternion> RotLerp;

  //        TransLerp.Init(theVectorOfTransStart,theVectorOfTransEnd);
  //        RotLerp.Init(startTrsf.GetRotation(),endTrsf.GetRotation());

  //        gp_Vec nowXYZ;
  //        gp_Quaternion nowRot;
  //        TransLerp.Interpolate(t,nowXYZ);
  //        qDebug()<<"2";
  //        //RotLerp.Interpolate(t,nowRot);
  //        qDebug()<<"3";

  //        gp_Trsf nowTrsf;
  //        nowTrsf.SetTranslation(nowXYZ);
  //        nowTrsf.SetRotation(nowRot);
  //        m_context->SetLocation(getAISShape(),startTrsf);
  //        m_context->Update(getAISShape(),Standard_True);
  //    }


  /*double ddd[6]{ 1,0,0,0,0,0 };
  for (int k = 0; k < 100; k++) {
    qDebug() << ddd[0];
    ddd[0] += 1;
    SetModelLocation_Euler(this->getAISShape(), ddd);
    QThread::msleep(100);
  }



  TopoDS_Shape S;
  gp_Trsf *theTransformation = new gp_Trsf();
  gp_Ax3 *ax3_1 = new gp_Ax3(*new gp_Pnt(0, 0, 0), *new gp_Dir(0, 0, 1));  //左手坐标系
  gp_Ax3 *ax3_2 = new gp_Ax3(*new gp_Pnt(60, 60, 60), *new gp_Dir(1, 1, 1));


  theTransformation->SetDisplacement(*ax3_1, *ax3_2);*/


  //    BRepBuilderAPI_Transform myBRepTransformation =new BRepBuilderAPI_Transform(S, theTransformation, false);
  //    TopoDS_Shape TransformedShape = myBRepTransformation->Shape();
}

// 刷新页面
void OccView::visual_update() {
  //QT计时器：每10ms刷新显示
  m_context->UpdateCurrentViewer();
}

void OccView::SetModelLocation(Handle(AIS_Shape) & aShape, gp_Trsf trsf) {
  Handle_AIS_InteractiveObject Current(aShape);
  if (!Current.IsNull()) {

    m_context->SetLocation(Current, trsf);
    m_context->Update(Current, Standard_True);//等价于这句话 myContext->UpdateCurrentViewer();//窗口更新
  }
}

//设置当前对象的位置
void OccView::SetModelLocation_Matrix(Handle(AIS_Shape) & aShape, double *matrixTemp) {


  cameraStart = getView()->Camera();
  gp_Trsf trsf;

  //    auto axe = new gp_Ax1(*new gp_Pnt(200, 60, 60), *new gp_Dir(0.0, 1.0, 0.0));//指定旋转轴
  //    trsf.SetTranslation(*new gp_Pnt(200, 60, 60),*new gp_Pnt(201, 60, 60));
  trsf.SetValues(matrixTemp[0], matrixTemp[1], matrixTemp[2], matrixTemp[3],
                 matrixTemp[4], matrixTemp[5], matrixTemp[6], matrixTemp[7],
                 matrixTemp[8], matrixTemp[9], matrixTemp[10], matrixTemp[11]);
  qDebug() << "1";
  SetModelLocation(aShape, trsf);
}

//通过YPR角度设置当前对象的位置
void OccView::SetModelLocation_Euler(Handle(AIS_Shape) & aShape, double *pTemp) {

  auto sourceTrsf = m_context->Location(aShape);
  double Rx{pTemp[0]}, Ry{pTemp[1]}, Rz{pTemp[2]};
  Rx = Rx / 180 * M_PI;
  Ry = Ry / 180 * M_PI;
  Rz = Rz / 180 * M_PI;

  //设置欧拉角
  gp_Trsf aTrsf_Rotation;
  gp_Quaternion aQ;
  aQ.SetEulerAngles(gp_YawPitchRoll, Rx, Ry, Rz);
  aTrsf_Rotation.SetRotation(aQ);

  //设置平移向量
  gp_Trsf aTrsf_Translation;
  gp_Vec theVectorOfTrans(pTemp[3], pTemp[4], pTemp[5]);
  aTrsf_Translation.SetTranslation(theVectorOfTrans);
  gp_Trsf trsf = aTrsf_Translation * aTrsf_Rotation;
  SetModelLocation(aShape, trsf);
}

void OccView::angleDebug(const gp_Ax3 &FromSystem, const gp_Ax3 &ToSystem)//变换前后的坐标系
{
  gp_Trsf trsf;
  trsf.SetTransformation(FromSystem, ToSystem);
  gp_Quaternion quaternion = trsf.GetRotation();//获取四元数，存储了旋转信息
  //gp_Mat mat=quaternion.GetMatrix () ;//获取旋转矩阵
  Standard_Real theAlpha, theBeta, theGamma;
  //从四元数中获取欧拉角，一共有24种，根据需要添加
  quaternion.GetEulerAngles(gp_Intrinsic_XYZ, theAlpha, theBeta, theGamma);
  //与PowerMill中每个数值差一个负号，输出角度
  qDebug() << "position:" << trsf.TranslationPart().X() << "," << trsf.TranslationPart().Y() << ","
           << trsf.TranslationPart().Z();
  qDebug() << "Angle:" << -theAlpha * 180 / 3.14 << "," << -theBeta * 180 / 3.14 << "," << -theGamma * 180 / 3.14
           << endl;
}

void OccView::PartMoveSim() {
}

void OccView::RobotMoveSim() {
}

void OccView::toolTrihedronDisplay() {


#ifdef ER100
  gp_Trsf aCubeTrsf, aCubeTrsf3, aCubeTrsf4, aCubeTrsf5, aCubeTrsf6;

  gp_Trsf rot, tran;
  gp_Vec gv1(gp_Dir(0, 0, 1)), gv12(-GeneralAx6.Direction());
  gp_Quaternion quaternion;
  quaternion.SetRotation(gv1, gv12);
  tran.SetRotation(quaternion);

  gp_Vec loc;
  loc.SetX(GeneralAx6.Location().X());
  loc.SetY(GeneralAx6.Location().Y());
  loc.SetZ(GeneralAx6.Location().Z());
  rot.SetTranslationPart(loc);
  aCubeTrsf = rot * tran;

  tool0Ax2.Transform(aCubeTrsf);
  Handle(Geom_Axis2Placement) TrihedronAxis = new Geom_Axis2Placement(tool0Ax2);
  toolTrihedron = new AIS_Trihedron(TrihedronAxis);
  toolTrihedron->SetDatumPartColor(Prs3d_DP_XAxis, Quantity_NOC_RED2);
  toolTrihedron->SetDatumPartColor(Prs3d_DP_YAxis, Quantity_NOC_GREEN2);
  toolTrihedron->SetDatumPartColor(Prs3d_DP_ZAxis, Quantity_NOC_BLUE2);
  toolTrihedron->SetLabel(Prs3d_DP_XAxis, "X");
  toolTrihedron->SetLabel(Prs3d_DP_YAxis, "Y");
  toolTrihedron->SetLabel(Prs3d_DP_ZAxis, "Z");
  m_context->Display(toolTrihedron, Standard_True);

  RobotAISShape[6]->AddChild(toolTrihedron);


#endif// ER100
}

void OccView::RobotBackHome() {
  getJoint01CurrentAngle() = getJoint02CurrentAngle() = getJoint03CurrentAngle() = 0;
  getJoint04CurrentAngle() = getJoint05CurrentAngle() = getJoint06CurrentAngle() = 0;
  gp_Ax1 ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
  gp_Trsf trans;
  trans.SetRotation(GeneralAx1, getJoint01CurrentAngle());
  m_context->SetLocation(RobotAISShape[1], trans);

  gp_Ax1 ax2(gp_Pnt(260, 0, 675), gp_Dir(0, 1, 0));
  trans.SetRotation(GeneralAx2, getJoint02CurrentAngle());
  m_context->SetLocation(RobotAISShape[2], trans);

  gp_Ax1 ax3(gp_Pnt(260, 0, 1355), gp_Dir(0, 1, 0));
  trans.SetRotation(GeneralAx3, getJoint03CurrentAngle());
  m_context->SetLocation(RobotAISShape[3], trans);

  gp_Ax1 ax4(gp_Pnt(662, 0, 1320), gp_Dir(1, 0, 0));
  trans.SetRotation(GeneralAx4, getJoint04CurrentAngle());
  m_context->SetLocation(RobotAISShape[4], trans);

  gp_Ax1 ax5(gp_Pnt(930, 0, 1320), gp_Dir(0, 1, 0));
  trans.SetRotation(GeneralAx5, getJoint05CurrentAngle());
  m_context->SetLocation(RobotAISShape[5], trans);

  gp_Ax1 ax6(gp_Pnt(1088, 0, 1320), gp_Dir(1, 0, 0));
  trans.SetRotation(GeneralAx6, getJoint06CurrentAngle());
  m_context->SetLocation(RobotAISShape[6], trans);

  m_context->UpdateCurrentViewer();
}

void OccView::CameraAnaminationStart() {
}

void OccView::ButtonAxis01MoveForward() {
  gp_Trsf trans;
  getJoint01CurrentAngle() = getJoint01CurrentAngle() + deltaAngle;
  auto angle = getJoint01CurrentAngle() - Joint01OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx1, getJoint01CurrentAngle());
  m_context->SetLocation(RobotAISShape[1], trans);
  m_context->UpdateCurrentViewer();

  qDebug() << "CurrentAngle:" << getJoint01CurrentAngle() * 180 / PI_OCC << ","
           << getJoint02CurrentAngle() * 180 / PI_OCC << "," << getJoint03CurrentAngle() * 180 / PI_OCC << ","
           << getJoint04CurrentAngle() * 180 / PI_OCC << "," << getJoint05CurrentAngle() * 180 / PI_OCC << ","
           << getJoint06CurrentAngle() * 180 / PI_OCC;
}

void OccView::ButtonAxis02MoveForward() {
  gp_Trsf trans;
  getJoint02CurrentAngle() = getJoint02CurrentAngle() + deltaAngle;
  auto angle = getJoint02CurrentAngle() - Joint02OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx2, getJoint02CurrentAngle());
  m_context->SetLocation(RobotAISShape[2], trans);
  m_context->UpdateCurrentViewer();
  qDebug() << "CurrentAngle:" << getJoint01CurrentAngle() * 180 / PI_OCC << ","
           << getJoint02CurrentAngle() * 180 / PI_OCC << "," << getJoint03CurrentAngle() * 180 / PI_OCC << ","
           << getJoint04CurrentAngle() * 180 / PI_OCC << "," << getJoint05CurrentAngle() * 180 / PI_OCC << ","
           << getJoint06CurrentAngle() * 180 / PI_OCC;
}

void OccView::ButtonAxis03MoveForward() {
  gp_Trsf trans;
  getJoint03CurrentAngle() = getJoint03CurrentAngle() + deltaAngle;
  auto angle = getJoint03CurrentAngle() - Joint03OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx3, getJoint03CurrentAngle());
  m_context->SetLocation(RobotAISShape[3], trans);
  m_context->UpdateCurrentViewer();
  qDebug() << "CurrentAngle:" << getJoint01CurrentAngle() * 180 / PI_OCC << ","
           << getJoint02CurrentAngle() * 180 / PI_OCC << "," << getJoint03CurrentAngle() * 180 / PI_OCC << ","
           << getJoint04CurrentAngle() * 180 / PI_OCC << "," << getJoint05CurrentAngle() * 180 / PI_OCC << ","
           << getJoint06CurrentAngle() * 180 / PI_OCC;
}

void OccView::ButtonAxis04MoveForward() {
  gp_Trsf trans;
  getJoint04CurrentAngle() = getJoint04CurrentAngle() + deltaAngle;
  auto angle = getJoint04CurrentAngle() - Joint04OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx4, getJoint04CurrentAngle());
  m_context->SetLocation(RobotAISShape[4], trans);
  m_context->UpdateCurrentViewer();
  qDebug() << "CurrentAngle:" << getJoint01CurrentAngle() * 180 / PI_OCC << ","
           << getJoint02CurrentAngle() * 180 / PI_OCC << "," << getJoint03CurrentAngle() * 180 / PI_OCC << ","
           << getJoint04CurrentAngle() * 180 / PI_OCC << "," << getJoint05CurrentAngle() * 180 / PI_OCC << ","
           << getJoint06CurrentAngle() * 180 / PI_OCC;
}

void OccView::ButtonAxis05MoveForward() {
  gp_Trsf trans;
  getJoint05CurrentAngle() = getJoint05CurrentAngle() + deltaAngle;
  auto angle = getJoint05CurrentAngle() - Joint05OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx5, getJoint05CurrentAngle());
  m_context->SetLocation(RobotAISShape[5], trans);
  m_context->UpdateCurrentViewer();
  qDebug() << "CurrentAngle:" << getJoint01CurrentAngle() * 180 / PI_OCC << ","
           << getJoint02CurrentAngle() * 180 / PI_OCC << "," << getJoint03CurrentAngle() * 180 / PI_OCC << ","
           << getJoint04CurrentAngle() * 180 / PI_OCC << "," << getJoint05CurrentAngle() * 180 / PI_OCC << ","
           << getJoint06CurrentAngle() * 180 / PI_OCC;
}

void OccView::ButtonAxis06MoveForward() {
  gp_Trsf trans;
  getJoint06CurrentAngle() = getJoint06CurrentAngle() + deltaAngle;
  auto angle = getJoint06CurrentAngle() - Joint06OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx6, getJoint06CurrentAngle());
  m_context->SetLocation(RobotAISShape[6], trans);
  m_context->UpdateCurrentViewer();
  qDebug() << "CurrentAngle:" << getJoint01CurrentAngle() * 180 / PI_OCC << ","
           << getJoint02CurrentAngle() * 180 / PI_OCC << "," << getJoint03CurrentAngle() * 180 / PI_OCC << ","
           << getJoint04CurrentAngle() * 180 / PI_OCC << "," << getJoint05CurrentAngle() * 180 / PI_OCC << ","
           << getJoint06CurrentAngle() * 180 / PI_OCC;
}

void OccView::ButtonAxis01MoveBackward() {
  gp_Trsf trans;
  getJoint01CurrentAngle() = getJoint01CurrentAngle() - deltaAngle;
  auto angle = getJoint01CurrentAngle() - Joint01OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx1, getJoint01CurrentAngle());
  m_context->SetLocation(RobotAISShape[1], trans);
  m_context->UpdateCurrentViewer();
}

void OccView::ButtonAxis02MoveBackward() {
  gp_Trsf trans;
  getJoint02CurrentAngle() = getJoint02CurrentAngle() - deltaAngle;
  auto angle = getJoint02CurrentAngle() - Joint02OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx2, getJoint02CurrentAngle());
  m_context->SetLocation(RobotAISShape[2], trans);
  m_context->UpdateCurrentViewer();
}

void OccView::ButtonAxis03MoveBackward() {
  gp_Trsf trans;
  getJoint03CurrentAngle() = getJoint03CurrentAngle() - deltaAngle;
  auto angle = getJoint03CurrentAngle() - Joint03OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx3, getJoint03CurrentAngle());
  m_context->SetLocation(RobotAISShape[3], trans);
  m_context->UpdateCurrentViewer();
}

void OccView::ButtonAxis04MoveBackward() {
  gp_Trsf trans;
  getJoint04CurrentAngle() = getJoint04CurrentAngle() - deltaAngle;
  auto angle = getJoint04CurrentAngle() - Joint04OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx4, getJoint04CurrentAngle());
  m_context->SetLocation(RobotAISShape[4], trans);
  m_context->UpdateCurrentViewer();
}

void OccView::ButtonAxis05MoveBackward() {
  gp_Trsf trans;
  getJoint05CurrentAngle() = getJoint05CurrentAngle() - deltaAngle;
  auto angle = getJoint05CurrentAngle() - Joint05OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx5, getJoint05CurrentAngle());
  m_context->SetLocation(RobotAISShape[5], trans);
  m_context->UpdateCurrentViewer();
}

void OccView::ButtonAxis06MoveBackward() {
  gp_Trsf trans;
  getJoint06CurrentAngle() = getJoint06CurrentAngle() - deltaAngle;
  auto angle = getJoint06CurrentAngle() - Joint06OriginAngle;
  Ui::PILimit(angle);
  trans.SetRotation(GeneralAx6, getJoint06CurrentAngle());
  m_context->SetLocation(RobotAISShape[6], trans);
  m_context->UpdateCurrentViewer();
}
//
//void OccView::setLinkPM(std::array<double, 7 * 16> link_pm) {
//  gp_Trsf transformation;
//  for (int i = 1; i < 7; ++i) {
//    transformation.SetValues(link_pm[i * 16], link_pm[i * 16 + 1], link_pm[i * 16 + 2], link_pm[i * 16 + 3] * 1000,
//                             link_pm[i * 16 + 4], link_pm[i * 16 + 5], link_pm[i * 16 + 6], link_pm[i * 16 + 7] * 1000,
//                             link_pm[i * 16 + 8], link_pm[i * 16 + 9], link_pm[i * 16 + 10],
//                             link_pm[i * 16 + 11] * 1000);
//    //std::cout << "position:" << "x " << link_pm[i * 16 + 3] << " y " << link_pm[i * 16 + 7] << " z "<< link_pm[i * 16 + 11] << std::endl;
//    m_context->SetLocation(RobotAISShape[i], transformation);
//  }
//}

void OccView::setLinkPQ(std::array<double, 7 * 7> link_pq) {
  gp_Trsf transformation;
  for (int i = 1; i < 7; ++i) {
    transformation.SetTransformation(
            gp_Quaternion(link_pq[i * 7 + 3], link_pq[i * 7 + 4], link_pq[i * 7 + 5], link_pq[i * 7 + 6]),
            gp_Vec(link_pq[i * 7] * 1000, link_pq[i * 7 + 1] * 1000, link_pq[i * 7 + 2] * 1000));
    m_context->SetLocation(RobotAISShape[i], transformation);
  }
  //  std::cout<<"test"<<std::endl;
}

//
//void OccView::setAngle(double angle) {
//  gp_Trsf trans1, trans2, trans3, trans4, trans5, trans6;
//  trans1.SetRotation(GeneralAx1, angle);
//  trans2.SetRotation(GeneralAx2, angle);
//  trans3.SetRotation(GeneralAx3, angle);
//  trans4.SetRotation(GeneralAx4, angle);
//  trans5.SetRotation(GeneralAx5, angle);
//  trans6.SetRotation(GeneralAx6, angle);
//
//  m_context->SetLocation(RobotAISShape[1], trans1);
//  m_context->SetLocation(RobotAISShape[2], trans2);
//  m_context->SetLocation(RobotAISShape[3], trans3);
//  m_context->SetLocation(RobotAISShape[4], trans4);
//  m_context->SetLocation(RobotAISShape[5], trans5);
//  m_context->SetLocation(RobotAISShape[6], trans6);
//  m_context->UpdateCurrentViewer();
//}
//
//
//void OccView::setAngle(double *angle) {
//  gp_Trsf trans1, trans2, trans3, trans4, trans5, trans6;
//  trans1.SetRotation(GeneralAx1, angle[0]);
//  trans2.SetRotation(GeneralAx2, angle[1]);
//  trans3.SetRotation(GeneralAx3, angle[2]);
//  trans4.SetRotation(GeneralAx4, angle[3]);
//  trans5.SetRotation(GeneralAx5, angle[4]);
//  trans6.SetRotation(GeneralAx6, angle[5]);
//
//  getJoint01CurrentAngle() = angle[0];
//  getJoint02CurrentAngle() = angle[1];
//  getJoint03CurrentAngle() = angle[2];
//  getJoint04CurrentAngle() = angle[3];
//  getJoint05CurrentAngle() = angle[4];
//  getJoint06CurrentAngle() = angle[5];
//
//  m_context->SetLocation(RobotAISShape[1], trans1);
//  m_context->SetLocation(RobotAISShape[2], trans2);
//  m_context->SetLocation(RobotAISShape[3], trans3);
//  m_context->SetLocation(RobotAISShape[4], trans4);
//  m_context->SetLocation(RobotAISShape[5], trans5);
//  m_context->SetLocation(RobotAISShape[6], trans6);
//  m_context->UpdateCurrentViewer();
//  qDebug() << "slot:" << angle[0];
//}

void OccView::setVisiable(nameState name, bool state) {
  if (name == nameState::joint1) {
    if (state == true) {
      m_context->Display(RobotAISShape[1], Standard_True);
      m_context->Display(RobotAISShape[0], Standard_True);
    } else {
      m_context->Erase(RobotAISShape[1], Standard_True);
      m_context->Erase(RobotAISShape[0], Standard_True);
    }
  }

  if (name == nameState::tool) {
    if (state == true) {
      m_context->Display(ToolAISShape, Standard_True);
    } else {
      m_context->Erase(ToolAISShape, Standard_True);
    }
  }
}

void OccView::setExternalToolEnable(bool enable) {
  ExternalToolEnable = enable;
}

void OccView::ButtonPartCoorOK() {
  Eigen::Matrix4d originMatrix;
  double origin_pe[6]{originPart0Coordinate.x, originPart0Coordinate.y, originPart0Coordinate.z,
                      originPart0Coordinate.rx, originPart0Coordinate.ry, originPart0Coordinate.rz};
  Ui::s_pe2pm(origin_pe, originMatrix, "123");
  qDebug() << "originPart0Coordinate:" << originPart0Coordinate.x << "," << originPart0Coordinate.y << ","
           << originPart0Coordinate.z << "," << originPart0Coordinate.rx << "," << originPart0Coordinate.ry << ","
           << originPart0Coordinate.rz;
  Eigen::Matrix4d currentMatrix;
  double current_pe[6]{getPartCoor().x, getPartCoor().y, getPartCoor().z, getPartCoor().rx, getPartCoor().ry,
                       getPartCoor().rz};
  Ui::s_pe2pm(current_pe, currentMatrix, "123");


  double jointTool0[6]{getJoint01CurrentAngle() * PI_OCC / 180,
                       getJoint02CurrentAngle() * PI_OCC / 180,
                       getJoint03CurrentAngle() * PI_OCC / 180,
                       getJoint04CurrentAngle() * PI_OCC / 180,
                       getJoint05CurrentAngle() * PI_OCC / 180,
                       getJoint06CurrentAngle() * PI_OCC / 180};

  robot_tool0_matrix = Ui::ESTUN_ER100_3000_MDH_Forward(jointTool0);

  robot_tool0_matrix(0, 3) = robot_tool0_matrix(0, 3) * 1000;
  robot_tool0_matrix(1, 3) = robot_tool0_matrix(1, 3) * 1000;
  robot_tool0_matrix(2, 3) = robot_tool0_matrix(2, 3) * 1000;


  Ui::qDebugMatrix4d(robot_tool0_matrix, "robot_tool0_matrix");
  auto transMatrix = robot_tool0_matrix * currentMatrix * originMatrix.inverse();
  gp_Trsf transpart;
  double pe_00[6]{0.0};
  Ui::s_pm2pe(transMatrix, pe_00, "123");
  gp_Trsf trans_00, rot_00;

  qDebug() << "ButtonPartCoorOK:"
           << "pe_00:";
  qDebug() << pe_00[0];
  qDebug() << pe_00[1];
  qDebug() << pe_00[2];

  //qDebug() << pe_00[3]*180/PI;
  //qDebug() << pe_00[4] * 180 / PI;
  //qDebug() << pe_00[5] * 180 / PI;

  trans_00.SetTranslationPart(gp_Vec(pe_00[0], pe_00[1], pe_00[2]));
  gp_Quaternion origin_qua;
  origin_qua.SetEulerAngles(gp_EulerSequence::gp_Intrinsic_XYZ, pe_00[3], pe_00[4], pe_00[5]);
  rot_00.SetRotation(origin_qua);
  transpart = trans_00 * rot_00;

  m_context->SetLocation(partTrihedron, transpart);
  m_context->Update(partTrihedron, true);
}

void OccView::ButtonToolCoorOK() {
  gp_Ax1 ax1(gp_Pnt(930, 0, 1300), gp_Dir(0, 1, 0));
  gp_Trsf trans, rot, transs;

  trans.SetTranslationPart(gp_Vec(getToolCoor().x, getToolCoor().y, getToolCoor().z));
  gp_Quaternion qua;
  qua.SetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, getToolCoor().rx, getToolCoor().ry, getToolCoor().rz);
  rot.SetRotation(qua);
  transs = trans * rot;//先平移再按照当前的矩阵旋转；
  //transs = rot * trans;//先平移再按照世界坐标系旋转；


  //trans.SetTranslationPart(gp_Vec(getToolCoor().x - originTool0Coordinate.x, getToolCoor().y - originTool0Coordinate.y, getToolCoor().z - originTool0Coordinate.z));
  //gp_Quaternion qua;
  //qua.SetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, getToolCoor().rx - originTool0Coordinate.rx, getToolCoor().ry - originTool0Coordinate.ry, getToolCoor().rz - originTool0Coordinate.rz);
  //rot.SetRotation(qua);
  //transs = rot * trans;


  //gp_Trsf current_trans, current_rot, current_transs;
  //gp_Trsf origin_trans, origin_rot, origin_transs;
  //current_trans.SetTranslationPart(gp_Vec(getToolCoor().x, getToolCoor().y, getToolCoor().z));
  //gp_Quaternion current_qua;
  //current_qua.SetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, getToolCoor().rx, getToolCoor().ry, getToolCoor().rz);
  //current_rot.SetRotation(current_qua);
  //current_transs = current_trans * current_rot;

  //origin_trans.SetTranslationPart(gp_Vec(originTool0Coordinate.x, originTool0Coordinate.y, originTool0Coordinate.z));
  //gp_Quaternion origin_qua;
  //origin_qua.SetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, originTool0Coordinate.rx, originTool0Coordinate.ry, originTool0Coordinate.rz);
  //origin_rot.SetRotation(origin_qua);
  //origin_transs = origin_trans * origin_rot;

  //auto origin_matrix = Ui::transf2Matrix(origin_transs);
  //auto current_matrix = Ui::transf2Matrix(current_transs);


  gp_Trsf transpart;
  Eigen::Matrix4d originMatrix;
  double origin_pe[6]{originTool0Coordinate.x, originTool0Coordinate.y, originTool0Coordinate.z,
                      originTool0Coordinate.rx, originTool0Coordinate.ry, originTool0Coordinate.rz};
  Ui::s_pe2pm(origin_pe, originMatrix, "123");
  qDebug() << "originTool0Coordinate:" << originTool0Coordinate.x << "," << originTool0Coordinate.y << ","
           << originTool0Coordinate.z << "," << originTool0Coordinate.rx << "," << originTool0Coordinate.ry << ","
           << originTool0Coordinate.rz;
  Eigen::Matrix4d currentMatrix;
  double current_pe[6]{getToolCoor().x, getToolCoor().y, getToolCoor().z, getToolCoor().rx, getToolCoor().ry,
                       getToolCoor().rz};
  Ui::s_pe2pm(current_pe, currentMatrix, "123");

  auto transMatrix = currentMatrix * originMatrix.inverse();

  Ui::qDebugMatrix4d(currentMatrix, "currentMatrix");
  Ui::qDebugMatrix4d(originMatrix, "originMatrix");
  Ui::qDebugMatrix4d(originMatrix.inverse(), "originMatrix.inverse()");
  Ui::qDebugMatrix4d(transMatrix, "transMatrix");

  double pe_00[6];
  Ui::s_pm2pe(transMatrix, pe_00, "123");
  gp_Trsf trans_00, rot_00;
  //trans_00.SetTranslationPart(gp_Vec(getToolCoor().x - originTool0Coordinate.x, getToolCoor().y - originTool0Coordinate.y, getToolCoor().z - originTool0Coordinate.z));
  trans_00.SetTranslationPart(gp_Vec(pe_00[0], pe_00[1], pe_00[2]));
  gp_Quaternion origin_qua;
  origin_qua.SetEulerAngles(gp_EulerSequence::gp_Intrinsic_XYZ, pe_00[3], pe_00[4], pe_00[5]);
  rot_00.SetRotation(origin_qua);
  transpart = trans_00 * rot_00;
  m_context->SetLocation(toolTrihedron, transpart);

  qDebug() << "deltaX:" << pe_00[0];
  qDebug() << "deltaY:" << pe_00[1];
  qDebug() << "deltaZ:" << pe_00[2];
  qDebug() << "deltaRX:" << pe_00[3];
  qDebug() << "deltaRY:" << pe_00[4];
  qDebug() << "deltaRZ:" << pe_00[5];

  m_context->Update(toolTrihedron, true);

  //gp_Trsf current_trans, current_rot, current_transs;
  //current_trans.SetTranslationPart(gp_Vec(getToolCoor().x, getToolCoor().y, getToolCoor().z));
  //gp_Quaternion current_qua;
  //current_qua.SetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, getToolCoor().rx, getToolCoor().ry, getToolCoor().rz);
  //current_rot.SetRotation(current_qua);
  //current_transs = current_trans * current_rot;

  //gp_Trsf transpart = current_transs;


  //qDebug() << " transpart.TranslationPart():" << transpart.TranslationPart().X() << "," << transpart.TranslationPart().Y() << "," << transpart.TranslationPart().Z();
  //double RX, RY, RZ;
  //transpart.GetRotation().GetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, RX, RY, RZ);

  //qDebug() << " transpart.TranslationPart():" << RX << "," << RY << "," << RZ;

  //double Rx, Ry, Rz;

  //gp_Trsf current_trans00, current_rot00, current_transs00;
  //current_trans00.SetTranslationPart(gp_Vec(-originTool0Coordinate.x, -originTool0Coordinate.y, -originTool0Coordinate.z));
  //gp_Quaternion current_qua00;
  //current_qua00.SetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, - originTool0Coordinate.rx, - originTool0Coordinate.ry, - originTool0Coordinate.rz);
  //current_rot00.SetRotation(current_qua00);
  //current_transs00 = current_rot00* current_trans00;

  //auto loc = m_context->Location(toolTrihedron);
  //loc.Transformation().GetRotation().GetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, Rx, Ry, Rz);
  //qDebug() << loc.Transformation().TranslationPart().X() << loc.Transformation().TranslationPart().Y() << loc.Transformation().TranslationPart().Z();
  //qDebug() << Rx; qDebug() << Ry; qDebug() << Rz;
}


void OccView::startSelectFirstCurve() {
  if (m_context->HasDetectedShape()) {
    TopoDS_Shape abc = m_context->DetectedShape();
    if (abc.ShapeType() == TopAbs_ShapeEnum::TopAbs_EDGE) {
      auto V = TopoDS::Edge(abc);
      ++num_FC;
      std::string str_output = std::to_string(num_FC) + " FirstCurve";
      QString qstr_output = QString::fromStdString(str_output);
      //存储选择的线
      PairfirstCurve.emplace_back(qstr_output, V);
      //显示选择成功文字
      emit firstCurveCompleteSigal();
      selectFirstCurve = false;
    }
  }
}

void OccView::startSelectSecondCurve() {
  if (m_context->HasDetectedShape()) {
    TopoDS_Shape abc = m_context->DetectedShape();
    if (abc.ShapeType() == TopAbs_ShapeEnum::TopAbs_EDGE) {
      auto V = TopoDS::Edge(abc);
      ++num_SC;
      std::string str_output = std::to_string(num_SC) + " SecondCurve";
      QString qstr_output = QString::fromStdString(str_output);
      PairsecondCurve.emplace_back(qstr_output, V);
      emit secondCurveCompleteSigal();
      selectSecondCurve = false;
    }
  }
}

void OccView::startSelectPlains() {
  if (m_context->HasDetectedShape()) {
    TopoDS_Shape abc = m_context->DetectedShape();
    if (abc.ShapeType() == TopAbs_ShapeEnum::TopAbs_FACE) {
      auto V = TopoDS::Face(abc);
      ++num_P;
      std::string str_output = std::to_string(num_P) + " Plain";
      QString qstr_output = QString::fromStdString(str_output);
      PairPlains.emplace_back(qstr_output, V);
      emit faceSelectCompleteSigal();
      selectFaces = false;

      /*      //选择的线需要在面上面找到, 显示边界  todo：判断面是否存在
      auto aFace = TopoDS::Face(PairPlains.back().second);
      Handle(Geom_Surface) surf=BRep_Tool::Surface(aFace); // get surface properties
      for (TopExp_Explorer wireExp(aFace, TopAbs_WIRE); wireExp.More(); wireExp.Next()){
        for (TopExp_Explorer edgeExp(wireExp.Current(), TopAbs_EDGE); edgeExp.More(); edgeExp.Next()){
          TopoDS_Edge aFaceEdge = TopoDS::Edge(edgeExp.Current());
          //绘制边界
          //      TopoDS_Edge aFaceEdge = BRepBuilderAPI_MakeEdge(anEdge, 0, 20);
          Handle(AIS_Shape) aFaceEdgeais_shape = new AIS_Shape(aFaceEdge);
          aFaceEdgeais_shape->SetColor(Quantity_NOC_YELLOW);
          m_context->Display(aFaceEdgeais_shape, Standard_True);
          // Next step
          std::cout<<"edge"<<std::endl;
        }
      }*/
    }
  }
}

void OccView::ButtonFirstCurve() {
  //失效选择模式
  m_context->Deactivate();
  //开启线选择模式
  auto aSubShapeSelMode = AIS_Shape::SelectionMode(TopAbs_EDGE);
  m_context->Activate(PartAISShape, aSubShapeSelMode);
  //启动选取第一条线到 PairfirstCurve 里
  selectFirstCurve = true;
}

void OccView::ButtonSecondCurve() {
  m_context->Deactivate();
  auto aSubShapeSelMode = AIS_Shape::SelectionMode(TopAbs_EDGE);
  m_context->Activate(PartAISShape, aSubShapeSelMode);
  selectSecondCurve = true;
}

void OccView::ButtonPlainSelect() {
  m_context->Deactivate();
  auto aSubShapeSelMode = AIS_Shape::SelectionMode(TopAbs_FACE);
  m_context->Activate(PartAISShape, aSubShapeSelMode);
  selectFaces = true;
}

//点坐标转换为面上的UV值
gp_Pnt2d OccView::FaceParameters(const TopoDS_Face &face, const gp_Pnt &pt) {
  // get face as surface
  const Handle(Geom_Surface) &surface = BRep_Tool::Surface(face);
  // create shape analysis object
  ShapeAnalysis_Surface sas(surface);
  // get UV of point on surface
  gp_Pnt2d uv = sas.ValueOfUV(pt, 0.01);
  // return parameters of point on face
  return uv;
}

//将边分割为点edge2points
auto OccView::E2P(TopoDS_Edge &E, const int i) -> std::vector<gp_Pnt> {
  Standard_Real curve_first, curve_last;
  Handle(Geom_Curve) curve = BRep_Tool::Curve(E, curve_first, curve_last);
  std::vector<double> params(i);
  std::vector<gp_Pnt> points(i);
  //为了获得每个点，对线取等分
  for (int k = 0; k < i; k++) {
    params[k] = (curve_last - curve_first) / (i - 1) * k + curve_first;
    curve->D0(params[k], points[k]);
    //std::cout<< " points["<<k<<"]:" << points[k].X() << " " << points[k].Y() << " " << points[k].Z()<<std::endl;
    //绘制点
    TopoDS_Vertex pvertex = BRepBuilderAPI_MakeVertex(points[k]);
    Handle(AIS_Shape) pshape = new AIS_Shape(pvertex);
    pshape->SetColor(Quantity_NOC_GOLD);
    m_context->Display(pshape, Standard_True);
    PartAISShape->AddChild(pshape);
  }
  return points;
}

//将边分割为点edge2UV
auto OccView::E2UV(const TopoDS_Edge &E, const TopoDS_Face &F, const int i) -> std::vector<gp_Pnt2d> {
  Standard_Real curve_first, curve_last;
  Handle(Geom_Curve) curve = BRep_Tool::Curve(E, curve_first, curve_last);
  std::vector<double> params(i);
  std::vector<gp_Pnt> points(i);
  std::vector<gp_Pnt2d> UV(i);
  //为了获得每个点，对线取等分
  for (int k = 0; k < i; k++) {
    params[k] = (curve_last - curve_first) / (i - 1) * k + curve_first;
    curve->D0(params[k], points[k]);
    //    std::cout<< " points["<<k<<"]:" << points[k].X() << " " << points[k].Y() << " " << points[k].Z()<<std::endl;
    //绘制点
    TopoDS_Vertex pvertex = BRepBuilderAPI_MakeVertex(points[k]);
    Handle(AIS_Shape) pshape = new AIS_Shape(pvertex);
    pshape->SetColor(Quantity_NOC_GOLD);
    m_context->Display(pshape, Standard_True);
    PartAISShape->AddChild(pshape);
    //点坐标转换为面上的UV值
    UV[k] = FaceParameters(F, points[k]);
    //    std::cout<<"uv"<<k<<": "<<"U:"<<UV[k].X()<<"  V:"<<UV[k].Y()<<std::endl;
  }
  return UV;
}

//将边分割为点VectorEdge2UV
auto OccView::VE2UV(const std::vector<std::pair<QString, TopoDS_Shape>> &VE, const TopoDS_Face &F, const int &i, bool *is_dir_X, bool *is_dir_Y) -> std::vector<gp_Pnt2d> {
  //计算边的长度->确定选择边的段数（即i的分配）
  Standard_Real totalLengthF{0.0};
  for (auto &v: VE) {
    auto edge_v = TopoDS::Edge(v.second);
    Standard_Real curve_first, curve_last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge_v, curve_first, curve_last);
    totalLengthF += GCPnts_AbscissaPoint::Length(GeomAdaptor_Curve(curve));
  }
  //      std::cout<<"totalLengthF"<<totalLengthF<<std::endl;
  //初始边 获得的线段转换为UV数组
  std::vector<gp_Pnt2d> uv_total;
  std::vector<double> vk;
  for (auto &v: VE) {
    auto edge_v = TopoDS::Edge(v.second);
    Standard_Real curve_first, curve_last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge_v, curve_first, curve_last);
    int num_i = floor(GCPnts_AbscissaPoint::Length(GeomAdaptor_Curve(curve)) / totalLengthF * i);
    if (num_i < 2) { num_i = 2; }
    std::cout << "num_i" << num_i << std::endl;
    std::vector<gp_Pnt2d> uv = E2UV(edge_v, F, num_i);
    uv_total.insert(uv_total.end(), uv.begin(), uv.end());
    //计算斜率 确定横向/纵向
    double k = (uv.back().Y() - uv.begin()->Y()) / (uv.back().X() - uv.begin()->X());
    vk.push_back(std::atan(k) * 180.0 / PI_OCC);
  }
  double sum = std::accumulate(std::begin(vk), std::end(vk), 0.0);
  double mean = sum / vk.size();//倾斜角均值
                                //    std::cout<<"mean"<<mean<<std::endl;
                                //  bool is_dir_X{false},is_dir_Y{false};
  if (mean > -45 && mean < 45) {
    (*is_dir_X) = true;
  } else {
    (*is_dir_Y) = true;
  }
  //      std::cout<<"is_dir_X"<<*is_dir_X<<std::endl;
  //      std::cout<<"is_dir_Y"<<*is_dir_Y<<std::endl;
  //重新排升序
  if (is_dir_X) {
    std::sort(uv_total.begin(), uv_total.end(), [](gp_Pnt2d a, gp_Pnt2d b) { return a.X() < b.X(); });
  } else {
    std::sort(uv_total.begin(), uv_total.end(), [](gp_Pnt2d a, gp_Pnt2d b) { return a.Y() < b.Y(); });
  }
  return uv_total;
}

//X平均值
auto OccView::Xmean(std::vector<gp_Pnt2d> p) -> Standard_Real {
  double sum{0};
  for (auto &i: p) {
    sum += i.X();
  }
  return sum / p.size();
}
//Y平均值
auto OccView::Ymean(std::vector<gp_Pnt2d> p) -> Standard_Real {
  double sum{0};
  for (auto &i: p) {
    sum += i.Y();
  }
  return sum / p.size();
}


//面上的边生成法向量
auto OccView::NormalFromEdge(TopoDS_Edge &E, TopoDS_Face &F, const int i) -> std::vector<Ui::PointsVector> {
  Standard_Real curve_first, curve_last;
  //TopoEdge转换成curve才能裁剪
  Handle(Geom_Curve) curve = BRep_Tool::Curve(E, curve_first, curve_last);
  std::vector<double> params(i);
  std::vector<gp_Pnt> points(i);
  gp_Vec vec1, vec2;
  std::vector<Ui::PointsVector> pointsVector;
  //为了获得每个点UV，对边界线取等分
  for (int k = 0; k < i; k++) {
    params[k] = (curve_last - curve_first) / (i - 1) * k + curve_first;
    curve->D2(params[k], points[k], vec1, vec2);
    //    std::cout<< " points["<<k<<"]:" << points[k].X() << " " << points[k].Y() << " " << points[k].Z()<<std::endl;
    //绘制点
    TopoDS_Vertex pvertex = BRepBuilderAPI_MakeVertex(points[k]);
    Handle(AIS_Shape) pshape = new AIS_Shape(pvertex);
    pshape->SetColor(Quantity_NOC_GOLD);
    m_context->Display(pshape, Standard_True);
    PartAISShape->AddChild(pshape);
    //点坐标转换为面上的UV值
    BRepGProp_Face analysisFace(F);
    gp_Pnt2d uv = FaceParameters(F, points[k]);
    //    std::cout<<"uv"<<uv.X()<<uv.Y()<<std::endl;
    gp_Vec norm;
    gp_Pnt midPoint;
    analysisFace.Normal(uv.X(), uv.Y(), midPoint, norm);//返回面参数点的坐标及其法向
    // 绘制法线
    gp_Lin normLine(midPoint, gp_Dir(norm));
    TopoDS_Edge anEdge = BRepBuilderAPI_MakeEdge(normLine, 0, 5);
    Handle(AIS_Shape) ais_shape = new AIS_Shape(anEdge);
    ais_shape->SetColor(Quantity_NOC_BLUE1);
    m_context->Display(ais_shape, Standard_True);
    //绘制切线
    //    gp_Lin lin_tag(midPoint, gp_Dir(vec1));
    //    TopoDS_Edge edge_tag = BRepBuilderAPI_MakeEdge(lin_tag, 0, 3);
    //    Handle(AIS_Shape) ais_lin_tag = new AIS_Shape(edge_tag);
    //    ais_lin_tag->SetColor(Quantity_NOC_GREEN2);
    //    m_context->Display(ais_lin_tag, Standard_True);

    Ui::PointsVector pvec;
    pvec.p1 = midPoint; pvec.v1 = norm; pvec.v2 = vec1;
    pointsVector.push_back(pvec);
  }
  return pointsVector;
}

/********CAM计算*************/
void OccView::ButtonPointsCal() {

  //选择的最后一个面判定
  auto aFace = TopoDS::Face(PairPlains.back().second);

  TopLoc_Location loca;
  Handle(Geom_Surface) Surface = BRep_Tool::Surface(aFace, loca);
  GeomAdaptor_Surface theGASurface(Surface);
  //OCC表示曲面的方法：Plane平面, Cylinder圆柱面, Cone圆锥面, Sphere球面, Torus圆环面,
  // BezierSurface贝塞尔面, BSplineSurfaceB样条曲面, SurfaceOfRevolution旋转曲面, SurfaceOfExtrusion线性拉伸面,
  // OtherSurface偏移曲面，（Rectangle Trim Surface 矩形裁剪曲面） 共十种曲面

  //确认面的朝向是外
  if (aFace.Orientation() == TopAbs_REVERSED) { aFace.Reversed(); }
//  BRepGProp_Face analysisFace(aFace);
//  Standard_Real umin, umax, vmin, vmax;
//  analysisFace.Bounds(umin, umax, vmin, vmax);//获取拓扑面的参数范围
//  qDebug() << " theGASurface.GetType():" << theGASurface.GetType();

  //判定为平面
  if (theGASurface.GetType() == GeomAbs_Plane) {
   /*  qDebug() << "surface is GeomAbs_Plane";

    //边切割段数
    const int i = 5;
    //pipe的半径
    const int r = 5;
    //选择的第一边
    auto first_edge = TopoDS::Edge(PairfirstCurve.back().second);
    //选择的第二边
    auto last_edge = TopoDS::Edge(PairsecondCurve.back().second);
    std::vector<gp_Pnt> points1 = E2P(first_edge, i);
    std::vector<gp_Pnt> points2 = E2P(last_edge, i);
    std::vector<gp_Pnt2d> uv1 = E2UV(first_edge, aFace, i);
    std::vector<gp_Pnt2d> uv2 = E2UV(last_edge, aFace, i);
    Standard_Integer num_shape_section{0};
    NormalFromEdge(first_edge, aFace, 5);
    do {
      //制作选择边的first_point形成的圆
      Standard_Real curve_first, curve_last;
      Handle(Geom_Curve) curve = BRep_Tool::Curve(first_edge, curve_first, curve_last);
      Standard_Real params[4];
      gp_Pnt points[4];
      //为了获得每个点UV，对边界线取等分
      for (int k = 0; k < 4; k++) {
        params[k] = (curve_last - curve_first) / 4 * k + curve_first;
        curve->D0(params[k], points[k]);
        //        std::cout<< " points["<<k<<"]:" << points[k].X() << " " << points[k].Y() << " " << points[k].Z()<<std::endl;
        //绘制点
        TopoDS_Vertex pvertex = BRepBuilderAPI_MakeVertex(points[k]);
        Handle(AIS_Shape) pshape = new AIS_Shape(pvertex);
        pshape->SetColor(Quantity_NOC_GOLD);
        m_context->Display(pshape, Standard_True);
        PartAISShape->AddChild(pshape);
        //点坐标转换为面上的UV值
        gp_Pnt2d uv = FaceParameters(aFace, points[k]);
        //        std::cout<<"uv"<<uv.X()<<uv.Y()<<std::endl;
      }
      //      std::cout<<"point"<<points[0].X()<<" "<<points[0].Y()<<" "<<points[0].Z()<<std::endl;

      //得到圆的方向和形状
      gp_Dir dir_cir(points[1].X() - points[0].X(), points[1].Y() - points[0].Y(), points[1].Z() - points[0].Z());
      gp_Circ circ_first(gp_Ax2(points[0], dir_cir), 2);

      // 得到半圆semi_cir
      gp_Dir dir_plane(points2[0].X() - points[0].X(), points2[0].Y() - points[0].Y(), points2[0].Z() - points[0].Z());
      gp_Pln trimmed_plane(points[0], dir_plane);
      gp_Dir dir_per = dir_cir ^ dir_plane;
      gp_Pnt pUp(points[0].X() + dir_per.X(), points[0].Y() + dir_per.Y(), points[0].Z() + dir_per.Z());
      gp_Pnt pDown(points[0].X() - dir_per.X(), points[0].Y() - dir_per.Y(), points[0].Z() - dir_per.Z());
      Handle(Geom_TrimmedCurve) semi_cir = GC_MakeArcOfCircle(circ_first, pDown, pUp, 1);

      TopoDS_Edge edge_cir = BRepBuilderAPI_MakeEdge(semi_cir);
      TopoDS_Wire wire_cir = BRepBuilderAPI_MakeWire(edge_cir);
      TopoDS_Shape pipe = BRepPrimAPI_MakePrism(wire_cir, dir_cir);//得到拉伸面
      //绘制pipe
      Handle(AIS_Shape) ais_pipe = new AIS_Shape(pipe);
      m_context->SetColor(ais_pipe, Quantity_NOC_GOLD, Standard_True);
      m_context->Display(ais_pipe, Standard_True);
      PartAISShape->AddChild(ais_pipe);

      //绘制plane
      Handle(Geom_Plane) gepln = new Geom_Plane(trimmed_plane);
      Handle(AIS_Plane) ais_plane = new AIS_Plane(gepln);
      m_context->SetColor(ais_plane, Quantity_NOC_GOLD, Standard_True);
      m_context->Display(ais_plane, Standard_True);
      PartAISShape->AddChild(ais_plane);
      //      std::cout<<"plane "<<std::endl;

      //拉伸面与选择的平面相交
      BRepAlgoAPI_Section section(aFace, pipe, 1);
      section.ComputePCurveOn2(true);
      section.Approximation(true);
      section.Build();
      auto shape_section = section.Shape();
      num_shape_section = shape_section.NbChildren();
      //绘制section
      Handle(AIS_Shape) ais_section = new AIS_Shape(shape_section);
      m_context->SetColor(ais_section, Quantity_NOC_BLUE2, Standard_True);
      m_context->Display(ais_section, Standard_True);
      PartAISShape->AddChild(ais_section);

      std::vector<gp_Pnt2d> compound_uv;
      std::vector<TopoDS_Edge> his_edge;
      his_edge.push_back(first_edge);
      //对得到的相交线shape_section操作生成法向量
      for (TopExp_Explorer e(shape_section, TopAbs_EDGE); e.More(); e.Next()) {
        TopoDS_Edge compound_edge = TopoDS::Edge(e.Current());
        NormalFromEdge(compound_edge, aFace, 5);
        first_edge = compound_edge;
      }
      //      std::cout<<num_shape_section<<std::endl;
    } while (num_shape_section != 0);*/
  }
  //判定为B样条曲面
  else if (theGASurface.GetType() == GeomAbs_BSplineSurface) {
    qDebug() << "surface is GeomAbs_BSplineSurface";

    //UV方法（平行曲线）     选择面+选择面的一边+选择面的对边
   /*  //边切割段数   横向
    const int i{20};
    //pipe的半径   整个面走几刀  纵向（选择两线之间的距离）
    const int r{10};
    //将边分割为点VectorEdge2UV
    bool is_dir_X{false},is_dir_Y{false};
    std::vector<gp_Pnt2d> uv1 = VE2UV(PairfirstCurve, aFace, i, &is_dir_X, &is_dir_Y);
    std::vector<gp_Pnt2d> uv2 = VE2UV(PairsecondCurve, aFace, i, &is_dir_X, &is_dir_Y);
    //当初始边的数量不等于末端边的数量
    if(uv1.size()>uv2.size()){
      for(int a =uv2.size();a<uv1.size();++a){
        uv2.push_back(uv2.back());
      }
    }else{
      for(int a =uv1.size();a<uv2.size();++a){
        uv1.push_back(uv1.back());
      }
    }
    //开始计算
    Standard_Real U, V;
    gp_Vec norm;
    gp_Pnt midPoint;
    for (int d = 0; d <= r; ++d) {
      std::vector<Ui::PointsVector> pointsVector;
      for (int k = 0; k < uv1.size(); k++) {
        double delta = double(d)/double(r);
        if(is_dir_Y && Xmean(uv1)>0.5){//右边
          U = uv1[k].X() - delta;
          V = uv1[k].Y();
//          if(U<uv2[k].X()){continue;}
        }else if(is_dir_Y && Xmean(uv1)<0.5){//左边
          U = uv1[k].X() + delta;
          V = uv1[k].Y()  ;
//          if(U>uv2[k].X()){continue;}
        }else if(is_dir_X && Ymean(uv1)>0.5){//上边
          U = uv1[k].X() ;
          V = uv1[k].Y() - delta ;
//          if(V<uv2[k].Y()){continue;}
        }else{//下边
          U = uv1[k].X() ;
          V = uv1[k].Y() + delta;
//          if(V>uv2[k].Y()){continue;}
        }
        //绘制法线BRepGProp_Face
        gp_Vec norm_before(norm);
        gp_Pnt point_before(midPoint);
        BRepGProp_Face analysisFace(aFace);
        analysisFace.Normal(U, V, midPoint, norm);//返回面参数中点的坐标及其法向
        gp_Lin normLine(midPoint, gp_Dir(norm));
        TopoDS_Edge anEdge = BRepBuilderAPI_MakeEdge(normLine, 0, 5);
        Handle(AIS_Shape) ais_shape = new AIS_Shape(anEdge);
        ais_shape->SetColor(Quantity_NOC_BLUE2);
        m_context->Display(ais_shape, Standard_True);
        //跳过第一点
        if(k!=0){
          gp_Vec vec1( point_before,midPoint);
          Ui::PointsVector pvec;
          pvec.p1 = point_before;  pvec.v1 = norm_before;  pvec.v2 = vec1;
          pointsVector.push_back(pvec);
        }
        //最后一点
        if(k==uv1.size()-1){
          gp_Vec vec1( point_before,midPoint);
          Ui::PointsVector pvec;
          pvec.p1 = midPoint;  pvec.v1 = norm;  pvec.v2 = vec1;
          pointsVector.push_back(pvec);
        }
      }
      pointVecVecs.push_back(pointsVector);
    }
*/
    //UV方法(两曲线渐进)   选择面+选择面的一边+选择面的对边
   /*  //边切割段数
    const int i{20};
    //pipe的半径   整个面走几刀
    const int r{10};
    //将边分割为点 VectorEdge2UV
    bool is_dir_X{false},is_dir_Y{false};
    std::vector<gp_Pnt2d> uv1 = VE2UV(PairfirstCurve, aFace, i, &is_dir_X, &is_dir_Y);
    std::vector<gp_Pnt2d> uv2 = VE2UV(PairsecondCurve, aFace, i, &is_dir_X, &is_dir_Y);
    //当初始边的数量不等于末端边的数量
    if(uv1.size()>uv2.size()){
      for(int a =uv2.size();a<uv1.size();++a){
        uv2.push_back(uv2.back());
      }
    }else{
      for(int a =uv1.size();a<uv2.size();++a){
        uv1.push_back(uv1.back());
      }
    }
    //开始计算
    int count{0};
    Standard_Real U, V;
    gp_Vec norm;
    gp_Pnt midPoint;
    for (int d = 0; d <= r; ++d) {
      std::vector<Ui::PointsVector> pointsVector;
      for (int k = 0; k < uv1.size(); k++) {
        U = uv1[k].X() + (uv2[k].X() - uv1[k].X()) * d / r;
        V = uv1[k].Y() + (uv2[k].Y() - uv1[k].Y()) * d / r;
        //绘制法线BRepGProp_Face
        gp_Vec norm_before(norm);
        gp_Pnt point_before(midPoint);
        BRepGProp_Face analysisFace(aFace);
        analysisFace.Normal(U, V, midPoint, norm);//返回面参数中点的坐标及其法向
        gp_Lin normLine(midPoint, gp_Dir(norm));
        TopoDS_Edge anEdge = BRepBuilderAPI_MakeEdge(normLine, 0, 5);
        Handle(AIS_Shape) ais_shape = new AIS_Shape(anEdge);
        ais_shape->SetColor(Quantity_NOC_BLUE2);
        m_context->Display(ais_shape, Standard_True);
        //第一点
        if(k!=0){
          gp_Vec vec1( point_before,midPoint);
          Ui::PointsVector pvec;
          pvec.p1 = point_before;  pvec.v1 = norm_before;  pvec.v2 = vec1;
          pointsVector.push_back(pvec);
        }
        //最后一点
        if(k==uv1.size()-1){
          gp_Vec vec1( point_before,midPoint);
          Ui::PointsVector pvec;
          pvec.p1 = midPoint;  pvec.v1 = norm;  pvec.v2 = vec1;
          pointsVector.push_back(pvec);
        }
      }
      pointVecVecs.push_back(pointsVector);
    }*/

    //垂直曲线   选择面+选择面的一边+选择面的对边
    /*//边切割段数   横向
    const int i{20};
    //pipe的半径   整个面走几刀  纵向（选择两线之间的距离）
    const int r{10};
    //选择的边分段
    Standard_Real param_curve_first,param_curve_last;
    auto edge_v = TopoDS::Edge(PairfirstCurve.back().second);
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge_v,param_curve_first, param_curve_last);
    Standard_Real params[i];
    gp_Pnt points[i];
    gp_Vec vec_der[i];
    for (int k = 0; k < i; k++) {
      params[k] = (param_curve_last - param_curve_first) * k / i  + param_curve_first;
      curve->D1(params[k], points[k],vec_der[k]);
      gp_Pln pln_der(points[k],gp_Dir(vec_der[k]));
      //垂直面和选择面 交线
      BRepAlgoAPI_Section section(aFace, pln_der, Standard_True);
      BRepAlgoAPI_Common com;
      section.ComputePCurveOn2(true);
      section.Approximation(true);
      section.Build();
      auto shape = section.Shape();
      //绘制相交线
      Handle(AIS_Shape) aishape = new AIS_Shape(shape);
      aishape->SetColor(Quantity_NOC_GREEN2);
      m_context->Display(aishape, true);
      PartAISShape->AddChild(aishape);
      std::vector<Ui::PointsVector> pointsVector;
      for (TopExp_Explorer e(shape, TopAbs_EDGE); e.More(); e.Next()){
        TopoDS_Edge edge_com = TopoDS::Edge(e.Current());
        //绘制法线
        pointsVector = NormalFromEdge(edge_com,aFace,r);
      }
      pointVecVecs.push_back(pointsVector);
    }*/

    //两曲面间流线加工     选择前后两面+选择切割的面
   /* //边切割段数   横向
    const int i{20};
    //pipe的半径   整个面走几刀  纵向（选择两线之间的距离）
    const int r{10};
    //得到两个面的中点和法线
    auto face_first = TopoDS::Face(PairPlains[0].second);
    auto face_second = TopoDS::Face(PairPlains[1].second);
    TopLoc_Location loc_first = face_first.Location();
    //拓扑面和几何曲面的方向相反时反转
    if (face_first.Orientation() == TopAbs_REVERSED) {
      face_first.Reversed();
    }
    BRepGProp_Face gp_face_first(face_first);
    Standard_Real umin_first, umax_first, vmin_first, vmax_first;
    gp_face_first.Bounds(umin_first, umax_first, vmin_first, vmax_first);//获取拓扑面的参数范围
    Standard_Real midU_first, midV_first;
    midU_first = (umin_first + umax_first) / 2;//拓扑面的参数中点
    midV_first = (vmin_first + vmax_first) / 2;
    gp_Vec norm_first;
    gp_Pnt midPoint_first;
    gp_face_first.Normal(midU_first, midV_first, midPoint_first, norm_first);//返回面参数中点的坐标及其法向
    //绘制法线
    gp_Lin normLine_first(midPoint_first, gp_Dir(norm_first));
    TopoDS_Edge anEdge_first = BRepBuilderAPI_MakeEdge(normLine_first, 0, 100);
    Handle(AIS_Shape) ais_shape_first = new AIS_Shape(anEdge_first);
    ais_shape_first->SetColor(Quantity_NOC_BLUE2);
    m_context->Display(ais_shape_first, Standard_True);

    //拓扑面和几何曲面的方向相反时反转
    if (face_second.Orientation() == TopAbs_REVERSED) {
      face_second.Reversed();
    }
    BRepGProp_Face gp_face_second(face_second);
    Standard_Real umin_second, umax_second, vmin_second, vmax_second;
    gp_face_second.Bounds(umin_second, umax_second, vmin_second, vmax_second);//获取拓扑面的参数范围
    Standard_Real midU_second, midV_second;
    midU_second = (umin_second + umax_second) / 2;//拓扑面的参数中点
    midV_second = (vmin_second + vmax_second) / 2;
    gp_Vec norm_second;
    gp_Pnt midPoint_second;
    gp_face_second.Normal(midU_second, midV_second, midPoint_second, norm_second);//返回面参数中点的坐标及其法向
    //绘制法线
    gp_Lin normLine_second(midPoint_second, gp_Dir(norm_second));
    TopoDS_Edge anEdge_second = BRepBuilderAPI_MakeEdge(normLine_second, 0, 100);
    Handle(AIS_Shape) ais_shape_second = new AIS_Shape(anEdge_second);
    ais_shape_second->SetColor(Quantity_NOC_GREEN1);
    m_context->Display(ais_shape_second, Standard_True);

    //求交点
    Standard_Real curve_lin_first1, curve_lin_last1, curve_lin_first2, curve_lin_last2;
    Handle(Geom_Curve) curve_line_first = BRep_Tool::Curve(anEdge_first, curve_lin_first1, curve_lin_last1);
    Handle(Geom_Curve) curve_line_second = BRep_Tool::Curve(anEdge_second, curve_lin_first2, curve_lin_last2);
    GeomAPI_ExtremaCurveCurve aExtCurve;
    aExtCurve.Init(curve_line_second, curve_line_first);
    Standard_Integer NbResults = aExtCurve.NbExtrema();
    gp_Pnt p1, p2;
    aExtCurve.Points(1, p1, p2);

    TColgp_Array1OfPnt Poles(1, 3);
    Poles.SetValue(1, midPoint_first);
    Poles.SetValue(2, p2);
    Poles.SetValue(3, midPoint_second);
    TColStd_Array1OfReal PolesWeight(1, 3);
    PolesWeight.SetValue(1, 1.0);
    PolesWeight.SetValue(2, 0.707);
    PolesWeight.SetValue(3, 1.0);
    for (int i = 0; i < 3; ++i) {
      TopoDS_Vertex aVertex = BRepBuilderAPI_MakeVertex(Poles.Value(i + 1));
      Handle(AIS_Shape) vert = new AIS_Shape(aVertex);
      m_context->SetColor(vert, Quantity_NOC_WHITE, Standard_False);
      m_context->Display(vert, Standard_False);

      if (i != 2) {
        TopoDS_Edge tedge = BRepBuilderAPI_MakeEdge(Poles.Value(i + 1), Poles.Value(i + 2));
        TopoDS_Wire twire = BRepBuilderAPI_MakeWire(tedge);
        Handle(AIS_Shape) awire = new AIS_Shape(twire);
        m_context->SetColor(awire, Quantity_NOC_WHITE, Standard_False);
        m_context->Display(awire, Standard_False);
      }
    }
    /// 有理B样条曲线   todo 构造线修改
    Standard_Integer degree(2);
    Standard_Integer PNum = 3;
    Standard_Integer KNum = PNum - 1;
    TColStd_Array1OfReal knots(1, KNum);
    for (int s = 0; s < KNum; ++s) {
      knots.SetValue(s + 1, s);
    }
    TColStd_Array1OfInteger mults(1, KNum);
    for (int s = 0; s < KNum; ++s) {
      if (s == 0 || s == KNum - 1) {
        mults.SetValue(s + 1, degree + 1);
      } else {
        mults.SetValue(s + 1, 1);
      }
    }
    Handle(Geom_BSplineCurve) curve_B = new Geom_BSplineCurve(Poles, PolesWeight, knots, mults, degree);
    TopoDS_Edge ed1 = BRepBuilderAPI_MakeEdge(curve_B);
    TopoDS_Wire wr1 = BRepBuilderAPI_MakeWire(ed1);

    Handle(AIS_Shape) red = new AIS_Shape(wr1);
    m_context->SetColor(red, Quantity_NOC_GREEN2, Standard_False);
    m_context->Display(red, Standard_False);

    Standard_Real curve_first, curve_last;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(ed1, curve_first, curve_last);
    double params;
    gp_Pnt points;
    gp_Vec vec1, vec2;
    for (int k = 0; k < r; k++) {
      params = (curve_last - curve_first) / (r - 1) * k + curve_first;
      curve->D2(params, points, vec1, vec2);
      gp_Pln pln_der(points, gp_Dir(vec1));
      //垂直面和选择面 交线
      BRepAlgoAPI_Section section(aFace, pln_der, Standard_True);
      BRepAlgoAPI_Common com;
      section.ComputePCurveOn2(true);
      section.Approximation(true);
      section.Build();
      auto shape = section.Shape();
      //绘制相交线
      Handle(AIS_Shape) aishape = new AIS_Shape(shape);
      aishape->SetColor(Quantity_NOC_GREEN2);
      m_context->Display(aishape, true);
      PartAISShape->AddChild(aishape);
      std::vector<Ui::PointsVector> pointsVector;
      for (TopExp_Explorer e(shape, TopAbs_EDGE); e.More(); e.Next()) {
        TopoDS_Edge edge_com = TopoDS::Edge(e.Current());
        //绘制法线
        pointsVector = NormalFromEdge(edge_com, aFace, i);
        pointVecVecs.push_back(pointsVector);
      }
    }*/

  }//判断曲面

  //计算切入切出
  ButtonPointsCutOverCal();
}


/********CAM添加切入切出*************/
void OccView::ButtonPointsCutOverCal() {

  qDebug() << "pointVecVecs.size():" << pointVecVecs.size();

  for (auto k = pointVecVecs.begin(); k != pointVecVecs.end(); k++) {
    //这个函数添加的唯一一句
    if(k->empty()){return;}

    std::vector<Ui::PointsVector> vectorpoints;

    //切入：
    gp_Dir In_N1(k->begin()->v2);
    gp_Dir In_V1(k->begin()->v1);
    gp_Vec In_vec = k->begin()->v2.Crossed(k->begin()->v1);
    gp_Dir In_vec_dir(In_vec / In_vec.Magnitude());
    gp_Pnt In_pnt;//圆心
    In_pnt.SetX(In_V1.X() * cutoverdata.radius + k->begin()->p1.X());
    In_pnt.SetY(In_V1.Y() * cutoverdata.radius + k->begin()->p1.Y());
    In_pnt.SetZ(In_V1.Z() * cutoverdata.radius + k->begin()->p1.Z());
    gp_Ax2 In_ax2(In_pnt, In_vec_dir, In_V1);
    gp_Circ In_Circle(In_ax2, cutoverdata.radius);
    TopoDS_Edge In_Edge_Circle = BRepBuilderAPI_MakeEdge(In_Circle, PI_OCC - cutoverdata.CutOverAngle, PI_OCC);
    Handle(AIS_Shape) In_CircleShape = new AIS_Shape(In_Edge_Circle);
    In_CircleShape->SetColor(Quantity_NOC_ORANGE);
    m_context->Display(In_CircleShape, Standard_True);
    PartAISShape->AddChild(In_CircleShape);

    /*******将切入分割成点**********/
    double aaFirst, aaLast;
    Handle(Geom_Curve) aacurve = BRep_Tool::Curve(In_Edge_Circle, aaFirst, aaLast);
    Handle(Geom_TrimmedCurve) aaTrimmedCurve = new Geom_TrimmedCurve(aacurve, aaFirst, aaLast);
    Handle(Geom_BSplineCurve) aaPCurve = GeomConvert::CurveToBSplineCurve(aaTrimmedCurve);
    Standard_Real aaafirst = aaPCurve->FirstParameter();
    Standard_Real aaalast = aaPCurve->LastParameter();
    for (int deta = 0; deta < cutoverdata.pointNum; deta++) {
      Standard_Real data = aaafirst + (aaalast - aaafirst) / cutoverdata.pointNum * deta;
      gp_Pnt pdata;
      gp_Vec vec1, vec2, vec3;
      aaPCurve->D3(data, pdata, vec1, vec2, vec3);
      TopoDS_Vertex pvertex = BRepBuilderAPI_MakeVertex(pdata);
      Handle(AIS_Shape) pshape = new AIS_Shape(pvertex);
      pshape->SetColor(Quantity_NOC_BLACK);
      m_context->Display(pshape, Standard_True);
      PartAISShape->AddChild(pshape);

      //切向量：
      gp_Lin normLine1(pdata, gp_Dir(vec1));
      TopoDS_Edge anEdge1 = BRepBuilderAPI_MakeEdge(normLine1, 0, 5);
      Handle(AIS_Shape) vecshape1 = new AIS_Shape(anEdge1);
      vecshape1->SetColor(Quantity_NOC_YELLOW);
      m_context->Display(vecshape1, Standard_True);
      PartAISShape->AddChild(vecshape1);

      //法向量（指向圆心）
      gp_Vec vec_nor(pdata, In_pnt);
      gp_Lin normLine2(pdata, gp_Dir(vec_nor));
      TopoDS_Edge anEdge2 = BRepBuilderAPI_MakeEdge(normLine2, 0, 5);
      Handle(AIS_Shape) vecshape2 = new AIS_Shape(anEdge2);
      vecshape2->SetColor(Quantity_NOC_BLUE1);
      m_context->Display(vecshape2, Standard_True);
      PartAISShape->AddChild(vecshape2);

      Ui::PointsVector dataPoint;
      dataPoint.p1 = pdata;
      dataPoint.v1 = vec_nor;
      dataPoint.v2 = vec1;
      vectorpoints.push_back(dataPoint);
    }

    for (auto m = k->begin(); m != k->end(); m++) {
      vectorpoints.push_back(*m);
    }

    //切出：
    gp_Dir Out_N1(k->back().v2);
    gp_Dir Out_V1(k->back().v1);
    gp_Vec Out_vec = k->back().v2.Crossed(k->back().v1);
    gp_Dir Out_vec_dir(Out_vec / Out_vec.Magnitude());
    gp_Pnt Out_pnt;//切出圆圆心
    Out_pnt.SetX(Out_V1.X() * cutoverdata.radius + k->back().p1.X());
    Out_pnt.SetY(Out_V1.Y() * cutoverdata.radius + k->back().p1.Y());
    Out_pnt.SetZ(Out_V1.Z() * cutoverdata.radius + k->back().p1.Z());
    gp_Ax2 Out_ax2(Out_pnt, Out_vec_dir, Out_V1);
    gp_Circ Out_Circle(Out_ax2, cutoverdata.radius);
    TopoDS_Edge Out_Edge_Circle = BRepBuilderAPI_MakeEdge(Out_Circle, PI_OCC, PI_OCC + cutoverdata.CutOverAngle);
    Handle(AIS_Shape) Out_CircleShape = new AIS_Shape(Out_Edge_Circle);
    Out_CircleShape->SetColor(Quantity_NOC_ORANGE);
    m_context->Display(Out_CircleShape, Standard_True);
    PartAISShape->AddChild(Out_CircleShape);

    /*******将切出分割成点**********/

    double ffFirst, ffLast;
    Handle(Geom_Curve) ffcurve = BRep_Tool::Curve(Out_Edge_Circle, ffFirst, ffLast);
    Handle(Geom_TrimmedCurve) ffTrimmedCurve = new Geom_TrimmedCurve(ffcurve, ffFirst, ffLast);
    Handle(Geom_BSplineCurve) ffPCurve = GeomConvert::CurveToBSplineCurve(ffTrimmedCurve);
    Standard_Real ffffirst = ffPCurve->FirstParameter();
    Standard_Real ffflast = ffPCurve->LastParameter();
    for (int deta = 0; deta < cutoverdata.pointNum; deta++) {
      Standard_Real data = ffffirst + (ffflast - ffffirst) / cutoverdata.pointNum * deta;
      gp_Pnt pdata;
      gp_Vec vec1, vec2, vec3;
      ffPCurve->D3(data, pdata, vec1, vec2, vec3);
      TopoDS_Vertex pvertex = BRepBuilderAPI_MakeVertex(pdata);
      Handle(AIS_Shape) pshape = new AIS_Shape(pvertex);
      pshape->SetColor(Quantity_NOC_BLACK);
      m_context->Display(pshape, Standard_True);
      PartAISShape->AddChild(pshape);

      //切向量：
      gp_Lin normLine1(pdata, gp_Dir(vec1));
      TopoDS_Edge anEdge1 = BRepBuilderAPI_MakeEdge(normLine1, 0, 5);
      Handle(AIS_Shape) vecshape1 = new AIS_Shape(anEdge1);
      vecshape1->SetColor(Quantity_NOC_YELLOW);
      m_context->Display(vecshape1, Standard_True);
      PartAISShape->AddChild(vecshape1);

      //法向量（指向圆心）
      gp_Vec vec_nor(pdata, Out_pnt);
      gp_Lin normLine2(pdata, gp_Dir(vec_nor));
      TopoDS_Edge anEdge2 = BRepBuilderAPI_MakeEdge(normLine2, 0, 5);
      Handle(AIS_Shape) vecshape2 = new AIS_Shape(anEdge2);
      vecshape2->SetColor(Quantity_NOC_BLUE1);
      m_context->Display(vecshape2, Standard_True);
      PartAISShape->AddChild(vecshape2);

      Ui::PointsVector dataPoint;
      dataPoint.p1 = pdata;
      dataPoint.v1 = vec_nor;
      dataPoint.v2 = vec1;
      vectorpoints.push_back(dataPoint);
    }

    CutOver_CAM_pointVecVecs.push_back(vectorpoints);
  }

  //  ButtonSaftyPlane();
}

void OccView::ConstructRobot(Assemly_Tree &tree) {
}

void OccView::InstallFilters(TopAbs_ShapeEnum shapeenum) {
  auto aSubShapeSelMode = AIS_Shape::SelectionMode(shapeenum);
  m_context->Activate(aSubShapeSelMode);
  qDebug() << "m_context->Activate(aSubShapeSelMode);";
}

void OccView::UninstallFilters(TopAbs_ShapeEnum shapeenum) {
  auto aSubShapeSelMode = AIS_Shape::SelectionMode(shapeenum);
  m_context->Deactivate(aSubShapeSelMode);
  qDebug() << "m_context->Deactivate(aSubShapeSelMode);";
}

void OccView::setJointAngle(nameState joint, double angle) {
  if (joint == nameState::joint1) {
    gp_Trsf trans;
    Ui::PILimit(angle);
    trans.SetRotation(GeneralAx1, angle);
    m_context->SetLocation(RobotAISShape[1], trans);
    m_context->UpdateCurrentViewer();

    Joint01CurrentAngle = angle;
  }

  if (joint == nameState::joint2) {
    gp_Trsf trans;
    Ui::PILimit(angle);
    trans.SetRotation(GeneralAx2, angle);
    m_context->SetLocation(RobotAISShape[2], trans);
    m_context->UpdateCurrentViewer();

    Joint02CurrentAngle = angle;
  }


  if (joint == nameState::joint3) {
    gp_Trsf trans;
    Ui::PILimit(angle);
    trans.SetRotation(GeneralAx3, angle);
    m_context->SetLocation(RobotAISShape[3], trans);
    m_context->UpdateCurrentViewer();

    Joint03CurrentAngle = angle;
  }

  if (joint == nameState::joint4) {
    gp_Trsf trans;
    Ui::PILimit(angle);
    trans.SetRotation(GeneralAx4, angle);
    m_context->SetLocation(RobotAISShape[4], trans);
    m_context->UpdateCurrentViewer();

    Joint04CurrentAngle = angle;
  }

  if (joint == nameState::joint5) {
    gp_Trsf trans;
    Ui::PILimit(angle);
    trans.SetRotation(GeneralAx5, angle);
    m_context->SetLocation(RobotAISShape[5], trans);
    m_context->UpdateCurrentViewer();

    Joint05CurrentAngle = angle;
  }

  if (joint == nameState::joint6) {
    gp_Trsf trans;
    Ui::PILimit(angle);
    trans.SetRotation(GeneralAx6, angle);
    m_context->SetLocation(RobotAISShape[6], trans);
    m_context->UpdateCurrentViewer();

    Joint06CurrentAngle = angle;
  }
}

//load robot
void OccView::ReadFile(QString aFilePath, Handle(Document) doc) {
  STEPCAFControl_Reader reader;
  reader.SetColorMode(true);
  reader.SetNameMode(true);
  reader.ReadFile(aFilePath.toUtf8());

  //  /*******注册progressbar***********/
  //  OccProgressIndicator *indicat = new OccProgressIndicator();
  //  QObject::connect(indicat, SIGNAL(updateProgress(int)), this, SLOT(importValue(int)));
  //  Handle_XSControl_WorkSession ws = reader.Reader().WS();
  //  ws->MapReader()->SetProgress(indicat);


  bool yes = reader.Transfer(doc);
  if (yes) {
    TDF_Label mainLabel = doc->Main();
    Handle(XCAFDoc_ShapeTool) ShapeTool = XCAFDoc_DocumentTool::ShapeTool(mainLabel);
    Handle(XCAFDoc_ColorTool) ColorTool = XCAFDoc_DocumentTool::ColorTool(mainLabel);
    {
      TDF_LabelSequence tdfLabels;
      ShapeTool->GetFreeShapes(tdfLabels);//获取装配体和组件对应名称
      NodeId RootNodeID = 0;
      TDF_Label Label = tdfLabels.Value(1);
      Ui::deepBuildAssemblyTree(RootNodeID, Label, doc->docTree);

      /******获取机器人的shape 到 RobotAISShape*****/

      TDF_LabelSequence components;
      XCAFDoc_ShapeTool::GetComponents(Label, components);
      for (int i = 1; i <= components.Length(); i++) {
        TDF_Label Label00 = components.Value(i);
        auto shape = ShapeTool->GetShape(Label00);
        RobotAISShape[i - 1] = new AIS_Shape(shape);
        m_context->Display(RobotAISShape[i - 1], true);
      }

      //RobotAISShape[5]->AddChild(RobotAISShape[6]);
      //RobotAISShape[4]->AddChild(RobotAISShape[5]);
      //RobotAISShape[3]->AddChild(RobotAISShape[4]);
      //RobotAISShape[2]->AddChild(RobotAISShape[3]);
      //RobotAISShape[1]->AddChild(RobotAISShape[2]);
      //RobotAISShape[0]->AddChild(RobotAISShape[1]);
    }
    initRobot();
  }
  m_view->FitAll();
}


/*
//import step
原文链接：https://blog.csdn.net/qq_15714933/article/details/108813205
*/

//void OccView::setTrackPoints() {
//
//}
