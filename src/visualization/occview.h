#ifndef OCCVIEW_H
#define OCCVIEW_H

#include <QWidget>
#include "qevent.h"
#include <QtWidgets/QApplication>
#include <QDebug>
#include <QMenu>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QEasingCurve>
#include <QThread>

#include <src/visualization/general.h>

#define BUTTON_ON true
#define BUTTON_OFF false

#ifndef OCC_VERSION_CHECK
#  define OCC_VERSION_CHECK(major, minor, patch) ((major<<16)|(minor<<8)|(patch))
#endif
#define OCC_VERSION_HEX    (OCC_VERSION_MAJOR << 16 | OCC_VERSION_MINOR << 8 | OCC_VERSION_MAINTENANCE)




struct Show_face
{
    int adv_face_index; //面的索引号
    TopoDS_Face face;
};


class OCAFBrowser
{
public:
    OCAFBrowser(Handle(TDocStd_Document) h)
        : pDoc(h)
    {
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


private:
    std::string toString(const TCollection_ExtendedString& extstr) const
    {
        char* str = new char[extstr.LengthOfCString() + 1];
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




class OccView : public QWidget
{
    Q_OBJECT
public:
    explicit OccView(QWidget *parent = nullptr);

signals:

public slots:

protected:
    //!三维场景转换模式
    enum CurrentAction3d
    {
        CurAction3d_Nothing,
        CurAction3d_DynamicPanning, //平移
        CurAction3d_DynamicZooming, //缩放
        CurAction3d_DynamicRotation //旋转
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

    Standard_Integer m_x_max;    //!记录鼠标平移坐标X
    Standard_Integer m_y_max;    //!记录鼠标平移坐标Y
    CurrentAction3d m_current_mode; //!三维场景转换模式(平移\缩放\旋转)
    TopoDS_Shape selectFaceShape;

public:
    QString workpiecePath,robotPath,toolPath,stlPath;
    //加载显示机器人模型
    void loadDisplayRobotWhole();


    //初始化机器人的初始位姿
    void initUR5();

    //加载显示机器人模型
    void loadDisplayRobotJoints();

    //加载和显示工件模型
    void loadDisplayWorkpiece();

    //加载和显示工具模型
    void loadDisplayTool();

    //加载和显示点云
    void loadDisplaySTL();

    const  Handle(V3d_View)& getView(){return m_view;}
    const  Handle(AIS_InteractiveContext)& getContext(){return m_context;}

    Handle_AIS_Shape& getAISShape(){return PartAISShape;}


    void SetModelLocation(Handle(AIS_Shape)& aShape,gp_Trsf trsf);
    void SetModelLocation_Euler(Handle(AIS_Shape)& aShape, double* pTemp);
    void SetModelLocation_Matrix(Handle(AIS_Shape)& aShape, double* matrixTemp);

    void angleDebug(const gp_Ax3& FromSystem, const gp_Ax3& ToSystem);

    void RobotBackHome();

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
    void ButtonTranXMoveForward();
    void ButtonTranYMoveForward();
    void ButtonTranZMoveForward();
    void ButtonTranRXMoveForward();
    void ButtonTranRYMoveForward();
    void ButtonTranRZMoveForward();
    void ButtonTranXMoveBackward();
    void ButtonTranYMoveBackward();
    void ButtonTranZMoveBackward();
    void ButtonTranRXMoveBackward();
    void ButtonTranRYMoveBackward();
    void ButtonTranRZMoveBackward();


    double& getJoint01CurrentAngle(){return Joint01CurrentAngle;}
    double& getJoint02CurrentAngle(){return Joint02CurrentAngle;}
    double& getJoint03CurrentAngle(){return Joint03CurrentAngle;}
    double& getJoint04CurrentAngle(){return Joint04CurrentAngle;}
    double& getJoint05CurrentAngle(){return Joint05CurrentAngle;}
    double& getJoint06CurrentAngle(){return Joint06CurrentAngle;}
    Ui::EularCoor& getPartCoor(){return part0Coordinate;}

private:
    void InitView();
    void InitFilters();
    Handle(StdSelect_FaceFilter) aFil1;
    Handle(StdSelect_FaceFilter) aFil2;
    QVector<Show_face>  workpiece_show_faces;   //必须放在struct Show_face后面
    QPoint m_prevPos;
    bool DrawLine{false};
    gp_Pnt myPntStart,myPntEnd;
    bool leftButON{false};
    bool faceSelect{false};
    std::vector<Handle(AIS_Shape)> NormalVector;
    Handle_Graphic3d_Camera cameraStart,cameraEnd;
    Handle(AIS_Shape)RobotAISShape[7];//机器人的shape
    Handle_AIS_Shape PartAISShape;//工件的shape
    Handle_AIS_Shape ToolAISShape;//工具的shape
    TopoDS_Shape PartTopoShape,ToolTopoShape;
    Handle(AIS_Trihedron) partTrihedron;//工件坐标系;
    Handle(AIS_Trihedron) toolTrihedron;//工具坐标系;
    gp_Ax2 tool0Ax2;
    gp_Ax2 part0Ax2;
    Ui::EularCoor part0Coordinate,originPart0Coordinate;
    Handle(Geom_Axis2Placement) PartTrihedronAxis;




    //右键菜单
private:
    QMenu*    m_contextMenu;
    QAction*  m_addAction;
    QAction*  m_delAction;
    gp_Trsf tttrsf;

    //关节转动
    static double Joint01CurrentAngle;
    static double Joint02CurrentAngle;
    static double Joint03CurrentAngle;
    static double Joint04CurrentAngle;
    static double Joint05CurrentAngle;
    static double Joint06CurrentAngle;

    double Joint01OriginAngle,Joint02OriginAngle,Joint03OriginAngle,Joint04OriginAngle,Joint05OriginAngle,Joint06OriginAngle;
    static double Joint01OriginAngle_static,Joint02OriginAngle_static,Joint03OriginAngle_static,Joint04OriginAngle_static,Joint05OriginAngle_static,Joint06OriginAngle_static;
    gp_Ax1 KukaAx1,KukaAx2,KukaAx3,KukaAx4,KukaAx5,KukaAx6;
    gp_Ax1 UR5Ax1,UR5Ax2,UR5Ax3,UR5Ax4,UR5Ax5,UR5Ax6;
    gp_Ax1 XB4Ax1,XB4Ax2,XB4Ax3,XB4Ax4,XB4Ax5,XB4Ax6;
    gp_Ax1 GeneralAx1,GeneralAx2,GeneralAx3,GeneralAx4,GeneralAx5,GeneralAx6;
    static double z;
    std::array<double,7*16> link_pm;

public:
    OCAFBrowser *ocaf;

};

#endif // OCCVIEW_H
