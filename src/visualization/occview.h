#ifndef OCCVIEW_H
#define OCCVIEW_H

#include <QDebug>
#include <QEasingCurve>
#include <QMenu>
#include <QThread>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QWidget>
#include <QtWidgets/QApplication>
#include <GC_MakeArcOfCircle.hxx>

#include "documents.h"
#include "general.h"
#include "qevent.h"
#include "threads.h"

#define BUTTON_ON true
#define BUTTON_OFF false

#ifndef OCC_VERSION_CHECK
#define OCC_VERSION_CHECK(major, minor, patch) ((major << 16) | (minor << 8) | (patch))
#endif
#define OCC_VERSION_HEX (OCC_VERSION_MAJOR << 16 | OCC_VERSION_MINOR << 8 | OCC_VERSION_MAINTENANCE)


struct Show_face {
  int adv_face_index;//面的索引号
  TopoDS_Face face;
};


class OCAFBrowser {
public:
  OCAFBrowser(Handle(TDocStd_Document) h)
      : pDoc(h) {
    myGroupIcon = QApplication::style()->standardIcon(QStyle::SP_DirIcon);

    TDataStd::IDList(myList);
    myList.Append(TDataStd_TreeNode::GetDefaultTreeID());
    myList.Append(TDataStd_Integer::GetID());
    myList.Append(TDocStd_Owner::GetID());
    myList.Append(TNaming_NamedShape::GetID());
    myList.Append(TNaming_UsedShapes::GetID());
    myList.Append(XCAFDoc_Color::GetID());
    myList.Append(XCAFDoc_ColorTool::GetID());
    myList.Append(XCAFDoc_LayerTool::GetID());
    myList.Append(XCAFDoc_ShapeTool::GetID());
    myList.Append(XCAFDoc_ShapeMapTool::GetID());
    myList.Append(XCAFDoc_Location::GetID());
  }

  void load(QTreeWidget *);

private:
  void load(const TDF_Label &label, QTreeWidgetItem *item, const QString &);
  std::string toString(const TCollection_ExtendedString &extstr) const {
    char *str = new char[extstr.LengthOfCString() + 1];
    extstr.ToUTF8CString(str);
    std::string text(str);
    delete[] str;
    return text;
  }

private:
  QIcon myGroupIcon;
  TDF_IDList myList;
  Handle(TDocStd_Document) pDoc;
};


class OccView : public QWidget {
  Q_OBJECT


public:
  explicit OccView(QWidget *parent = nullptr);
  ~OccView();

  enum nameState {
    joint1,
    joint2,
    joint3,
    joint4,
    joint5,
    joint6,
    tool,
    part,
  };


protected:
  //!三维场景转换模式
  enum CurrentAction3d {
    CurAction3d_Nothing,
    CurAction3d_DynamicPanning,//平移
    CurAction3d_DynamicZooming,//缩放
    CurAction3d_DynamicRotation//旋转
  };

  //!覆写绘图事件
  void paintEvent(QPaintEvent *);
  //!覆写窗口尺寸变化事件
  void resizeEvent(QResizeEvent *);
  //!返回窗口的绘制引擎
  QPaintEngine *paintEngine() const;
  //!覆写键盘按键按下事件
  //void keyPressEvent(QKeyEvent *event);
  //!覆写键盘按键释放事件
  //void keyReleaseEvent(QKeyEvent *event);
  //!覆写鼠标按键按下事件
  void mousePressEvent(QMouseEvent *event);
  //!覆写鼠标按键释放事件
  void mouseReleaseEvent(QMouseEvent *event);
  //!覆写鼠标移动事件
  void mouseMoveEvent(QMouseEvent *event);
  //!覆写鼠标滚轮事件
  void wheelEvent(QWheelEvent *event);


private:
  //!交互式上下文能够管理一个或多个查看器(viewer)中的图形行为和交互式对象的选择
  Handle(AIS_InteractiveContext) m_context;
  //!定义查看器(viewer)类型对象上的服务
  Handle(V3d_Viewer) m_viewer;
  //!创建一个视图
  Handle(V3d_View) m_view;
  //!创建3d接口定义图形驱动程序
  Handle(Graphic3d_GraphicDriver) m_graphic_driver;
  //存储STEP模型形状

  Standard_Integer m_x_max;      //!记录鼠标平移坐标X
  Standard_Integer m_y_max;      //!记录鼠标平移坐标Y
  CurrentAction3d m_current_mode;//!三维场景转换模式(平移\缩放\旋转)
  TopoDS_Shape selectFaceShape;

public:
  QString workpiecePath, robotPath, toolPath, stlPath;

  // 刷新页面
  void visual_update();

  //加载显示机器人模型
  void loadDisplayRobotWhole();

  void ReadFile(QString aFilePath, Handle(Document) doc);

  //初始化机器人的初始位姿
  void initRobot();

  //加载显示机器人模型
  void loadDisplayRobotJoints();

  //加载和显示工件模型
  void loadDisplayWorkpiece();

  //加载和显示工具模型
  void loadDisplayTool();

  //加载和显示点云
  void loadDisplaySTL();

  //显示法向量
  void displayNormalVector();

  void removeNormalVector();
  //提取工件中所有的面
  void pickUp(TopoDS_Shape);
  void getShape();
  void selectMode(Handle(AIS_Shape) selectmode, const Ui::selectionType &type);
  void DisselectMode(Handle(AIS_Shape) selectmode, const Ui::selectionType &type);
  void newPartCoordinate();
  void newToolCoordinate();
  void startSelectFirstCurve();
  void startSelectSecondCurve();
  void startSelectPlains();
  const Handle(V3d_View) & getView() {
    return m_view;
  }
  const Handle(AIS_InteractiveContext) & getContext() {
    return m_context;
  }
  bool &getDrawLine() {
    return DrawLine;
  }
  bool &getSelectstatus() {
    return faceSelect;
  }
  Handle_AIS_Shape &getAISShape() {
    return PartAISShape;
  }
  void setSelectType(const Ui::selectionType &type, const bool &selection) {
    if (faceSelect != selection) {
      if (selection) {
        selectMode(PartAISShape, type);
      } else {
        DisselectMode(PartAISShape, type);
      }
      faceSelect = selection;
    }
  }

  bool &getNewToolCoordinate() {
    return newToolCoordinateEnable;
  }
  bool &getNewPartCoordinate() {
    return newPartCoordinateEnable;
  }

  void initNewToolCoordinate() {
    firstPointSelected = secondPointSelected = thirdPointSelected = false;
  }

  void initNewPartCoordinate() {
    firstPointSelected = secondPointSelected = thirdPointSelected = false;
  }

  void coutPointCoor();
  gp_Pnt ChangeCoordinateSecond(double x, double y);
  void SetModelLocation(Handle(AIS_Shape) & aShape, gp_Trsf trsf);
  void SetModelLocation_Euler(Handle(AIS_Shape) & aShape, double *pTemp);
  void SetModelLocation_Matrix(Handle(AIS_Shape) & aShape, double *matrixTemp);
  void AnaminationStart();
  void angleDebug(const gp_Ax3 &FromSystem, const gp_Ax3 &ToSystem);
  void PartMoveSim();
  void RobotMoveSim();
  void toolTrihedronDisplay();
  void RobotBackHome();
  void CameraAnaminationStart();
  void ButtonAxis01MoveForward();
  void ButtonAxis02MoveForward();
  void ButtonAxis03MoveForward();
  void ButtonAxis04MoveForward();
  void ButtonAxis05MoveForward();
  void ButtonAxis06MoveForward();
  void ButtonAxis01MoveBackward();
  void ButtonAxis02MoveBackward();
  void ButtonAxis03MoveBackward();
  void ButtonAxis04MoveBackward();
  void ButtonAxis05MoveBackward();
  void ButtonAxis06MoveBackward();
  void ButtonPartCoorOK();
  void ButtonToolCoorOK();
  void ButtonFirstCurve();
  void ButtonSecondCurve();
  void ButtonPlainSelect();
  void ButtonPointsCal();


  double &getJoint01CurrentAngle() { return Joint01CurrentAngle; }
  double &getJoint02CurrentAngle() { return Joint02CurrentAngle; }
  double &getJoint03CurrentAngle() { return Joint03CurrentAngle; }
  double &getJoint04CurrentAngle() { return Joint04CurrentAngle; }
  double &getJoint05CurrentAngle() { return Joint05CurrentAngle; }
  double &getJoint06CurrentAngle() { return Joint06CurrentAngle; }
  Ui::EularCoor &getPartCoor() { return part0Coordinate; }
  Ui::EularCoor &getToolCoor() { return tool0Coordinate; }
  void ConstructRobot(Assemly_Tree &tree);
  void InstallFilters(TopAbs_ShapeEnum shapeenum);
  void UninstallFilters(TopAbs_ShapeEnum shapeenum);
  void setJointAngle(nameState joint, double angle);


private:
  void InitView();
  void InitFilters();
  Handle(StdSelect_FaceFilter) aFil1;
  Handle(StdSelect_FaceFilter) aFil2;
  QVector<Show_face> workpiece_show_faces;//必须放在struct Show_face后面
  QPoint m_prevPos;
  bool DrawLine{false};
  gp_Pnt myPntStart, myPntEnd;
  bool leftButON{false};
  bool faceSelect{false};
  std::vector<Handle(AIS_Shape)> NormalVector;
  Handle_Graphic3d_Camera cameraStart, cameraEnd;
  Handle(AIS_Shape) RobotAISShape[7];//机器人的shape
  Handle_AIS_Shape PartAISShape;     //工件的shape
  Handle_AIS_Shape ToolAISShape;     //工具的shape
  TopoDS_Shape RobotTopoShape, PartTopoShape, ToolTopoShape;
  Handle(AIS_Trihedron) partTrihedron;//工件坐标系;
  Handle(AIS_Trihedron) toolTrihedron;//工具坐标系;
  gp_Ax2 tool0Ax2;
  gp_Ax2 part0Ax2;
  Ui::EularCoor part0Coordinate, originPart0Coordinate;
  Ui::EularCoor tool0Coordinate, originTool0Coordinate;
  Handle(Geom_Axis2Placement) PartTrihedronAxis;
  Handle(Geom_Axis2Placement) ToolTrihedronAxis;

  Eigen::Matrix4d robot_tool0_matrix;
  bool ExternalToolEnable{true};

  std::vector<std::pair<gp_Pnt, gp_Vec>> pointVecs;//轨迹点

  static std::vector<std::array<double, 6>> trackPoints /*{{1,1,1,1,1,1}}*/;//轨迹点
public:
  static std::vector<std::array<double, 6>> GetTrackPoints() noexcept { return trackPoints; }
  //定义判定是否完成法向量计算
  static bool finish_norm;


signals:
  void mouseMovedSignal(const QPoint &posMouseInView);
  void NewPartCoordinateCompleteSigal();
  void NewToolCoordinateCompleteSigal();
  void firstCurveCompleteSigal();
  void secondCurveCompleteSigal();
  void faceSelectCompleteSigal();
  void sendimportValueSigal(int);
  //    void setTrackPoints(std::vector<std::array<double,6>> );

private slots:
  //    void setLinkPM(std::array<double, 7 * 16> link_pm);
  void setLinkPQ(std::array<double, 7 * 7> link_pq);

  //    std::vector<std::array<double,6>> GetTrackPoints() noexcept {return trackPoints;}
  //  void setAngle(double angle);
  //	void setAngle(double* angle);
//  void importValue(int value) {
//    emit sendimportValueSigal(value);
//  }

  //右键菜单
private:
  QMenu *m_contextMenu;
  QAction *m_addAction;
  QAction *m_delAction;
  gp_Trsf tttrsf;


  //新建坐标系
  gp_Pnt pointO, pointX, pointY;
  bool newToolCoordinateEnable{false};
  bool newPartCoordinateEnable{false};
  bool firstPointSelected{false}, secondPointSelected{false}, thirdPointSelected{false};

  //关节转动
  static double Joint01CurrentAngle;
  static double Joint02CurrentAngle;
  static double Joint03CurrentAngle;
  static double Joint04CurrentAngle;
  static double Joint05CurrentAngle;
  static double Joint06CurrentAngle;

  double Joint01OriginAngle{0.0}, Joint02OriginAngle{0.0}, Joint03OriginAngle{0.0}, Joint04OriginAngle{0.0}, Joint05OriginAngle{0.0}, Joint06OriginAngle{0.0};
  static double Joint01OriginAngle_static, Joint02OriginAngle_static, Joint03OriginAngle_static, Joint04OriginAngle_static, Joint05OriginAngle_static, Joint06OriginAngle_static;
  gp_Ax1 KukaAx1, KukaAx2, KukaAx3, KukaAx4, KukaAx5, KukaAx6;
  gp_Ax1 UR5Ax1, UR5Ax2, UR5Ax3, UR5Ax4, UR5Ax5, UR5Ax6;
  gp_Ax1 ER100_3000Ax1, ER100_3000Ax2, ER100_3000Ax3, ER100_3000Ax4, ER100_3000Ax5, ER100_3000Ax6;
  gp_Ax1 XB4Ax1, XB4Ax2, XB4Ax3, XB4Ax4, XB4Ax5, XB4Ax6;
  gp_Ax1 GeneralAx1, GeneralAx2, GeneralAx3, GeneralAx4, GeneralAx5, GeneralAx6;

  std::array<double, 7 * 7> link_pq;

public:
  OCAFBrowser *ocaf;
  std::vector<std::pair<QString, TopoDS_Shape>> PairfirstCurve;
  std::vector<std::pair<QString, TopoDS_Shape>> PairsecondCurve;
  std::vector<std::pair<QString, TopoDS_Shape>> PairPlains;
  int num_FC{0};
  int num_SC{0};
  int num_P{0};
  bool selectFirstCurve{false};
  bool selectSecondCurve{false};
  bool selectFaces{false};
  //点坐标转换为面上的UV值
  gp_Pnt2d FaceParameters(const TopoDS_Face &face,const gp_Pnt &pt);
  //对面上的边生成法向量
  auto NormalFromEdge(TopoDS_Edge& E,TopoDS_Face& F,int i)->void;
  //将边分割为点edge2points
  auto OccView::E2P(TopoDS_Edge &E,const int i)->std::vector<gp_Pnt>;
  //将边分割为点edge2UV
  auto OccView::E2UV(const TopoDS_Edge &E,const TopoDS_Face& F,const int i)->std::vector<gp_Pnt2d>;
  //将边分割为点VectorEdge2UV
  auto OccView::VE2UV(const std::vector<std::pair<QString, TopoDS_Shape>>& VE, const TopoDS_Face& F,const int& i, bool* is_dir_X=false, bool* is_dir_Y=false )->std::vector<gp_Pnt2d>;
  //设置点为0或者1
  auto OccView::PointToPole(Standard_Real) -> Standard_Real;
  //计算直线斜率
  auto OccView::uv2atan(std::vector<gp_Pnt2d>) -> Standard_Real;
  //X平均值
  auto OccView::Xmean(std::vector<gp_Pnt2d>) -> Standard_Real;
  //Y平均值
  auto OccView::Ymean(std::vector<gp_Pnt2d>) -> Standard_Real;



public:
  void setVisiable(nameState name, bool state);
  void setExternalToolEnable(bool enable);
};

#endif// OCCVIEW_H
