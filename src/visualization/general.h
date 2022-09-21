#ifndef GENERAL_H
#define GENERAL_H

#include <QDebug>
#include <Eigen/Eigen>
#include <QTreeWidgetItem>

#include "button_flat.h"
#include "libtree.h"

namespace Ui {

#define deltaAngle 0.03
#define deltaLength 0.01
#define transPos 20
#define PI 3.1415926

static ButtonFlat* createViewBtn(QWidget* parent, const QIcon& icon, const QString& tooltip)
{
    auto btn = new ButtonFlat(parent);
    btn->setIcon(icon);
    btn->setIconSize(QSize(18, 18));
    btn->setFixedSize(24, 24);
    btn->setToolTip(tooltip);
    return btn;
}

typedef struct Joint {

	Joint() : j1(0), j2(0), j3(0), j4(0), j5(0), j6(0) {}

	Joint(double j1, double j2, double j3, double j4, double j5, double j6) : j1(j1), j2(j2), j3(j3), j4(j4), j5(j5), j6(j6) {}

	Joint(const Joint& joint)
	{
		j1 = joint.j1;
		j2 = joint.j2;
		j3 = joint.j3;
		j4 = joint.j4;
		j5 = joint.j5;
		j6 = joint.j6;
	}

	Joint& operator=(const Joint& joint)
	{
		j1 = joint.j1;
		j2 = joint.j2;
		j3 = joint.j3;
		j4 = joint.j4;
		j5 = joint.j5;
		j6 = joint.j6;
		return *this;
	}

	void print() const
	{

		qDebug() << QString("[%1, %2, %3, %4, %5, %6]").arg(j1).arg(j2).arg(j3).arg(j4).arg(j5).arg(j6);

	}

	double j1 = 0;      /// 单位角度
	double j2 = 0;
	double j3 = 0;
	double j4 = 0;
	double j5 = 0;
	double j6 = 0;

}Joint;

/** 关节类型 */
typedef enum JointType {
	BASE,   /// 基座
	J1,     /// 关节一
	J2,
	J3,
	J4,
	J5,
	J6

}JointType;


static gp_Pnt V3dView_to3dPosition(const Handle_V3d_View& view, double x, double y)
{
    double xEye, yEye, zEye, xAt, yAt, zAt;
    view->Eye(xEye, yEye, zEye);
    view->At(xAt, yAt, zAt);
    const gp_Pnt pntEye(xEye, yEye, zEye);
    const gp_Pnt pntAt(xAt, yAt, zAt);

    const gp_Vec vecEye(pntEye, pntAt);
    const bool vecEyeNotNull = vecEye.SquareMagnitude() > gp::Resolution();
    const gp_Dir dirEye(vecEyeNotNull ? vecEye : gp_Vec{0, 0, 1});

    const gp_Pln planeView(pntAt, dirEye);
    double px, py, pz;
    const int ix = static_cast<int>(std::round(x));
    const int iy = static_cast<int>(std::round(y));
    view->Convert(ix, iy, px, py, pz);
    const gp_Pnt pntConverted(px, py, pz);
    const gp_Pnt2d pntConvertedOnPlane = ProjLib::Project(planeView, pntConverted);
    return ElSLib::Value(pntConvertedOnPlane.X(), pntConvertedOnPlane.Y(), planeView);
}

static gp_Pnt ConvertClickToPoint(Standard_Real theX, Standard_Real theY, Handle(V3d_View) theView)
{
    Standard_Real XEye,YEye,ZEye,XAt,YAt,ZAt;
    theView->Eye(XEye,YEye,ZEye);
    theView->At(XAt,YAt,ZAt);
    gp_Pnt EyePoint(XEye,YEye,ZEye);
    gp_Pnt AtPoint(XAt,YAt,ZAt);

    gp_Vec EyeVector(EyePoint,AtPoint);
    gp_Dir EyeDir(EyeVector);

    gp_Pln PlaneOfTheView = gp_Pln(AtPoint,EyeDir);
    Standard_Real X,Y,Z;
    theView->Convert(int(theX),int(theY),X,Y,Z);
    gp_Pnt ConvertedPoint(X,Y,Z);
    gp_Pnt2d ConvertedPointOnPlane = ProjLib::Project(PlaneOfTheView,ConvertedPoint);

    gp_Pnt ResultPoint = ElSLib::Value(ConvertedPointOnPlane.X(),
                                       ConvertedPointOnPlane.Y(),
                                       PlaneOfTheView);
    return ResultPoint;
}

static void DrawLineByMouse(const gp_Pnt& thePntStart, const gp_Pnt& thePntEnd, Handle_AIS_Shape& myAISShape, Handle(AIS_InteractiveContext) &m_context)
{
    //检查传入参数
    if (thePntStart.IsEqual(thePntEnd, 1e-3))
        return;

    //构建拓扑线段
    Handle(Geom_TrimmedCurve) aSegment =
            GC_MakeSegment(thePntStart, thePntEnd);
    TopoDS_Edge aEdge = BRepBuilderAPI_MakeEdge(aSegment);
    static Handle(AIS_Shape)lastEdge;
    Handle(AIS_Shape)Edge=new AIS_Shape(aEdge);

    //将构建的拓扑线段设置至AIS_Shape形状中
    //   myAISShape->SetShape(aEdge);

    //    //移除前面绘画的旧线段, 绘制新线段。
    m_context->Remove(lastEdge, true);
    m_context->Display(myAISShape, FALSE);
    //m_context->Remove(myAISShape,true);
    m_context->Display(Edge, false);
    lastEdge=Edge;
    //更新View
    m_context->UpdateCurrentViewer();
}

static void qDebugMatrix3d(const Eigen::Matrix3d& RotateMatrix,const std::string& name){
    qDebug()<<name.c_str();
    qDebug()<<RotateMatrix(0,0)<<","<<RotateMatrix(0,1)<<","<<RotateMatrix(0,2);
    qDebug()<<RotateMatrix(1,0)<<","<<RotateMatrix(1,1)<<","<<RotateMatrix(1,2);
    qDebug()<<RotateMatrix(2,0)<<","<<RotateMatrix(2,1)<<","<<RotateMatrix(2,2);
}

static void qDebugMatrix4d(const Eigen::Matrix4d& RotateMatrix,const std::string& name){
    qDebug()<<name.c_str();
    auto data=[](const double&data_){
        if(data_<0.0001&&data_>-0.0001){
            return 0.0;
        }else{
            return data_;
        }
    };
    qDebug()<<data(RotateMatrix(0,0))<<","<<data(RotateMatrix(0,1))<<","<<data(RotateMatrix(0,2))<<","<<data(RotateMatrix(0,3));
    qDebug()<<data(RotateMatrix(1,0))<<","<<data(RotateMatrix(1,1))<<","<<data(RotateMatrix(1,2))<<","<<data(RotateMatrix(1,3));
    qDebug()<<data(RotateMatrix(2,0))<<","<<data(RotateMatrix(2,1))<<","<<data(RotateMatrix(2,2))<<","<<data(RotateMatrix(2,3));
    qDebug()<<data(RotateMatrix(3,0))<<","<<data(RotateMatrix(3,1))<<","<<data(RotateMatrix(3,2))<<","<<data(RotateMatrix(3,3));
}

static Eigen::Matrix4d Rotate2RotatePos(Eigen::Matrix3d RotateMatrix,double x,double y,double z)
{
    Eigen::Matrix4d RotatePosMatrix;
    RotatePosMatrix.block(0, 0, 3, 3) = RotateMatrix;
    RotatePosMatrix(0, 3) = x;
    RotatePosMatrix(1, 3) = y;
    RotatePosMatrix(2, 3) = z;
    RotatePosMatrix(3, 0) = 0;
    RotatePosMatrix(3, 1) = 0;
    RotatePosMatrix(3, 2) = 0;
    RotatePosMatrix(3, 3) = 1;
    return RotatePosMatrix;
}

static void s_pe2pm(const double (&pe)[6], Eigen::Matrix4d& matrix_c_n , std::string kind)
{

	if (kind == "123")
	{
		double thetax = pe[3];
		double thetay = pe[4];
		double thetaz = pe[5];
		Eigen::Vector3d eulerAngle(thetax, thetay, thetaz);//(Z-Y-X，即RPY)Eigen::Vector3d eulerAngle(yaw, PItch, roll)
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd PItchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::Matrix3d matrix_c;
		matrix_c = rollAngle * PItchAngle*yawAngle;
		matrix_c_n = Rotate2RotatePos(matrix_c, pe[0], pe[1], pe[2]);
	}

	if (kind == "321")
	{
		double thetax = pe[5];
		double thetay = pe[4];
		double thetaz = pe[3];
		Eigen::Vector3d eulerAngle(thetax, thetay, thetaz);//(Z-Y-X，即RPY)Eigen::Vector3d eulerAngle(yaw, PItch, roll)
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd PItchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::Matrix3d matrix_c;
		matrix_c = yawAngle * PItchAngle*rollAngle;
		matrix_c_n = Rotate2RotatePos(matrix_c, pe[0], pe[1], pe[2]);

	}
	if (kind == "323")
	{
		double thetax = pe[3];
		double thetay = pe[4];
		double thetaz = pe[5];
		Eigen::Vector3d eulerAngle(thetax, thetay, thetaz);//(Z-Y-X，即RPY)Eigen::Vector3d eulerAngle(yaw, PItch, roll)
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
		Eigen::AngleAxisd PItchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
		Eigen::Matrix3d matrix_c;
		matrix_c = rollAngle * PItchAngle*yawAngle;
		matrix_c_n = Rotate2RotatePos(matrix_c, pe[0], pe[1], pe[2]);

	}

}

static void getEulerFromTrsf(double* pe,const gp_Trsf& trsf){
    double dx,dy,dz,rx,ry,rz;
    dx= trsf.TranslationPart().X();
    dy= trsf.TranslationPart().Y();
    dz= trsf.TranslationPart().Z();
    trsf.GetRotation().GetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ,rx,ry,rz);
    pe[0]=dx;pe[1]=dy;pe[2]=dz;pe[3]=rx;pe[4]=ry;pe[5]=rz;
}

static void getMatrixFromTrsf(Eigen::Matrix4d& matrix,const gp_Trsf& trsf){
    double pe[6]{0};
    getEulerFromTrsf(pe,trsf);
    s_pe2pm(pe,matrix,"123");
}

static void s_pm_dot_pm(double* pm1,double* pm2, double* pm3)
{
    Eigen::Matrix4d m1,m2,m3;
    m1<<
         pm1[0],pm1[1],pm1[2],pm1[3],
            pm1[4],pm1[5],pm1[6],pm1[7],
            pm1[8],pm1[9],pm1[10],pm1[11],
            pm1[12],pm1[13],pm1[14],pm1[15];
    m2<<
         pm2[0],pm2[1],pm2[2],pm2[3],
            pm2[4],pm2[5],pm2[6],pm2[7],
            pm2[8],pm2[9],pm2[10],pm2[11],
            pm2[12],pm2[13],pm2[14],pm2[15];

    m3=m1*m2;

    pm3[0]=m3(0,0);pm3[1]=m3(0,1);pm3[2]=m3(0,2);pm3[3]=m3(0,3);
    pm3[4]=m3(1,0);pm3[5]=m3(1,1);pm3[6]=m3(1,2);pm3[7]=m3(1,3);
    pm3[8]=m3(2,0);pm3[9]=m3(2,1);pm3[10]=m3(2,2);pm3[11]=m3(2,3);
    pm3[12]=m3(3,0);pm3[13]=m3(3,1);pm3[14]=m3(3,2);pm3[15]=m3(3,3);
}

static void s_pm2pe(double* pm1, double* pe, std::string kind)
{
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d eulerAngle;
    rotation_matrix<<
                      pm1[0], pm1[1], pm1[2],
            pm1[4], pm1[5], pm1[6],
            pm1[8], pm1[9], pm1[10];
    if(kind=="321")
    {
        eulerAngle=rotation_matrix.eulerAngles(2,1,0);
    }
    if(kind=="123")
    {
        eulerAngle=rotation_matrix.eulerAngles(0,1,2);
    }

    pe[0]=pm1[3]; pe[1]=pm1[7]; pe[2]=pm1[11];
    pe[3]=eulerAngle(0); pe[4]=eulerAngle(1); pe[5]=eulerAngle(2);
}

static void s_pm2pe(const Eigen::Matrix4d& matrix,double* pe, std::string kind){
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d eulerAngle;
    rotation_matrix<<
                      matrix(0,0),  matrix(0,1),  matrix(0,2),
            matrix(1,0),  matrix(1,1),  matrix(1,2),
            matrix(2,0),  matrix(2,1),  matrix(2,2);

	if (kind == "123") {
		eulerAngle = rotation_matrix.eulerAngles(0, 1, 2);
	}
	if (kind == "321") {
		eulerAngle = rotation_matrix.eulerAngles(2, 1, 0);
	}
	if (kind == "323") {
		eulerAngle = rotation_matrix.eulerAngles(2, 1, 2);
	}


    pe[0]=matrix(0,3); pe[1]=matrix(1,3); pe[2]=matrix(2,3);
    pe[3]=eulerAngle(0); pe[4]=eulerAngle(1); pe[5]=eulerAngle(2);
}

struct robotDH{
    struct jointDH{
        double Angle_alfa_1,Length_a_1,Length_d,Angle_theta;
    };
    jointDH joint01dh,joint02dh,joint03dh,joint04dh,joint05dh,joint06dh;
};



static void PILimit(double& data){
    if(data>PI){
        data=data-2*PI;
    }
    if(data<-PI){
        data=data+2*PI;
    }
}

static void zeroLimit(double& data) {
	if (data < 0.00001&&data>-0.00001) {
		data = 0;
	}
}

/*******thetas 为弧度制 **********/
static Eigen::Matrix4d ESTUN_ER100_3000_MDH_Forward(const double* thetas)
{
	auto matixCal = [](double di, double ai, double afa, double thetaoffset, double jointtheta) {
		Eigen::Matrix4d res;

		auto theta = jointtheta + thetaoffset;

		res(0, 0) = std::cos(theta);                res(0, 1) = -std::sin(theta);                res(0, 2) = 0;                  res(0, 3) = ai;
		res(1, 0) = std::sin(theta)*std::cos(afa);  res(1, 1) = std::cos(theta)*std::cos(afa);   res(1, 2) = -std::sin(afa);     res(1, 3) = -di * std::sin(afa);
		res(2, 0) = std::sin(theta)*std::sin(afa);  res(2, 1) = std::cos(theta)*std::sin(afa);   res(2, 2) = std::cos(afa);      res(2, 3) = di * std::cos(afa);
		res(3, 0) = 0;                              res(3, 1) = 0;                               res(3, 2) = 0;                  res(3, 3) = 1;
		return res;
	};
	//qDebug() << "thetas:";
	//qDebug() << thetas[0] * 180 / PI;
	//qDebug() << thetas[1] * 180 / PI;
	//qDebug() << thetas[2] * 180 / PI;
	//qDebug() << thetas[3] * 180 / PI;
	//qDebug() << thetas[4] * 180 / PI;
	//qDebug() << thetas[5] * 180 / PI;





	double d1, d2, d3, d4, d5, d6;
	double a1, a2, a3, a4, a5, a6;
	double afa1, afa2, afa3, afa4, afa5, afa6;
	double thetaoffset1, thetaoffset2, thetaoffset3, thetaoffset4, thetaoffset5, thetaoffset6;

	d1 = 0.0;  	  d2 = 0.0;   	  d3 = 0.0;   	  d4 = -1.6;      d5 = 0.0;    	    d6 = 0.0;
	a1 = 0.0;     a2 = 0.26;      a3 = 1.15;      a4 = 0.23;      a5 = 0.0; 	    a6 = 0.0;
	afa1 = 0.0;   afa2 = PI / 2;  afa3 = 0.0;     afa4 = PI / 2;  afa5 = -PI / 2;   afa6 = PI / 2;
	thetaoffset1 = 0.0;  thetaoffset2 = -PI / 2;  thetaoffset3 = 0.0;  thetaoffset4 = 0.0;  thetaoffset5 = 0.0;   thetaoffset6 = 0.0;

	double pe_base2Axis0[6]{ 0.0,
							0,
							0.6455,
							180.0 / 180 * PI,
							0,
							0 };
	Eigen::Matrix4d matrix_base2Axis0;
	s_pe2pm(pe_base2Axis0, matrix_base2Axis0, "123");

	double pe_Axis02tool0[6]{ 0.0,
							 0.0,
							 -0.214,
							 0.0 / 180 * PI,
							 180.0 / 180 * PI,
							 0.0 };
	Eigen::Matrix4d matrix_Axis02tool0;
	s_pe2pm(pe_Axis02tool0, matrix_Axis02tool0, "123");


	auto matrixT1 = matixCal(d1, a1, afa1, thetaoffset1, thetas[0]);
	auto matrixT2 = matixCal(d2, a2, afa2, thetaoffset2, thetas[1]);
	auto matrixT3 = matixCal(d3, a3, afa3, thetaoffset3, thetas[2]);
	auto matrixT4 = matixCal(d4, a4, afa4, thetaoffset4, thetas[3]);
	auto matrixT5 = matixCal(d5, a5, afa5, thetaoffset5, thetas[4]);
	auto matrixT6 = matixCal(d6, a6, afa6, thetaoffset6, thetas[5]);

	auto res = matrixT1 * matrixT2*matrixT3*matrixT4*matrixT5*matrixT6;




	auto res01 = matrix_base2Axis0 * res * matrix_Axis02tool0;

	qDebugMatrix4d(res, "res_forward:");

	qDebugMatrix4d(res01, "res01_forward:");

	return res01;
}

static Assemly_Data GetData(const Handle(XCAFDoc_ShapeTool)& ShapeTool,
                            const Handle(XCAFDoc_ColorTool)& ColorTool,
                            const TDF_Label & Label,
                            TopLoc_Location Location)
{
    Assemly_Data data;

    if(!ShapeTool->IsAssembly(Label)){
        data.isAssembly=false;
        //data.shape=ShapeTool->GetShape(Label);
        data.name=ShapeTool->get_type_name();
    }
    else {
        data.isAssembly=true;
        //data.shape=ShapeTool->GetShape(Label);
        data.name=ShapeTool->get_type_name();
    }

    return  data;
}

static void MakeTree(const Handle(XCAFDoc_ShapeTool)& ShapeTool,
                     const Handle(XCAFDoc_ColorTool)& ColorTool,
                     const TDF_Label & Label,
                     TopLoc_Location Location,
                     Assemly_Node * ParentNode,
                     Assemly_Tree & Tree)
{
    TDF_LabelSequence components;
    if (ShapeTool->GetComponents(Label, components))
    {

        qDebug()<<"components.Length()"<<components.Length();

        for (Standard_Integer compIndex = 1; compIndex <= components.Length(); ++compIndex)
        {
            TDF_Label ChildLabel = components.Value(compIndex);
            if (ShapeTool->IsReference(ChildLabel))
            {
                TDF_Label ShapeLabel;
                if (ShapeTool->GetReferredShape(ChildLabel, ShapeLabel))
                {
                    qDebug()<<"true"<<","<<"do";
                    TopLoc_Location LocalLocation = Location * ShapeTool->GetLocation(ChildLabel);
                    Assemly_Data AssemlyData = GetData(ShapeTool, ColorTool, ShapeLabel, LocalLocation);
                    Assemly_Node * Node = Tree.AddNode(ParentNode, AssemlyData);
                    MakeTree(ShapeTool, ColorTool, ShapeLabel, LocalLocation, Node, Tree);
                }
            }
        }
    }
}


static void deepBuildAssemblyTree(NodeId parentNode, const TDF_Label& label,Assemly_Tree & tree)
{
    auto shapeComponents=[](const TDF_Label& lbl){
        TDF_LabelSequence seq;
        XCAFDoc_ShapeTool::GetComponents(lbl, seq);
        return seq;
    };

//    auto shapeReferred=[](const TDF_Label& lbl){
//        TDF_Label referred;
//        XCAFDoc_ShapeTool::GetReferredShape(lbl, referred);
//        return referred;
//    };

    qDebug()<<"parentNode:"<<parentNode;
    NodeId node = tree.appendChildID(parentNode, label);

    qDebug()<<"NodeId node:"<<node;
    if (XCAFDoc_ShapeTool::IsAssembly(label)) {
        for (const TDF_Label& child : shapeComponents(label)){
            deepBuildAssemblyTree(node, child,tree);
        }
    }

}

static void traverseTree_preOrder(NodeId id, const Assemly_Tree& tree, const std::function<void(NodeId itNodeId)>& callback)
{
    if (!tree.isNodeDeleted(id)) {
        callback(id);
        for (auto it = tree.nodeChildFirst(id); it != 0; it = tree.nodeSiblingNext(it))
            traverseTree_preOrder(it, tree, callback);
    }
}

static void traverseTree_preOrder(const Assemly_Tree& tree, const std::function<void(NodeId itNodeId)>& callback) {
    for (NodeId id : tree.m_vecRoot)
        traverseTree_preOrder(id, tree, callback);
}

static void traverseTree(NodeId id, const Assemly_Tree& tree, const std::function<void(NodeId itNodeId)>& callback) {
    return traverseTree_preOrder(id, tree, callback);
}


struct cafutils
{

    static  QString to_QString(const TCollection_AsciiString& str) {
        return QString::fromUtf8(str.ToCString(), str.Length());
    }

    static  const TCollection_ExtendedString& labelAttrStdName(const TDF_Label& label)
    {
        Handle_TDataStd_Name attrName;
        if (label.FindAttribute(TDataStd_Name::GetID(), attrName)) {
            return attrName->Get();
        }
        else {
            static const TCollection_ExtendedString nullStr = {};
            return nullStr;
        }
    }

    static  QString referenceItemText(const TDF_Label& instanceLabel, const TDF_Label& productLabel)
    {
        const QString instanceName = to_QString(labelAttrStdName(instanceLabel)).trimmed();
        const QString productName = to_QString(labelAttrStdName(productLabel)).trimmed();
        //const QByteArray strTemplate = Module::toInstanceNameTemplate(m_module->instanceNameFormat);
        QByteArray strTemplate;
        QString itemText = QString::fromUtf8(strTemplate);
        itemText.replace("%instance", instanceName)
                .replace("%product", productName);
        return itemText;
    }

    static QIcon shapeIcon(const TDF_Label& label)
    {
        if (XCAFDoc_ShapeTool::IsAssembly(label))
            return QIcon(":/themes/dark/stop.svg");
        else if (XCAFDoc_ShapeTool::IsReference(label))
            return QIcon(":/images/xde_reference_16.png"); // TODO move in Theme
        else if (XCAFDoc_ShapeTool::IsSimpleShape(label))
            return QIcon(":/themes/dark/stop.svg");

        return QIcon();
    }

};

enum selectionType{
    FaceSel=1,
    EdgeSel,
    VetextSel,
};

struct EularCoor{
    double x,y,z,rx,ry,rz;
};

struct robPosData
{
	double X{ 0 }, Y{ 0 }, Z{ 0 }, A{ 0 }, B{ 0 }, C{ 0 };
};


static void normalize(Eigen::Vector3d& vec)
{

	double n = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
	vec[0] /= n;
	vec[1] /= n;
	vec[2] /= n;

}

/* 角度值转弧度值 */
static inline double toRadian(double degree)
{
	return (degree / 180.0)* PI;
}

static inline double toDegree(double radian)
{
	return (radian / PI) * 180.0;
}

static Eigen::Matrix4d transf2Matrix(const gp_Trsf& trans)
{


	double X = trans.TranslationPart().X();
	double Y = trans.TranslationPart().Y();
	double Z = trans.TranslationPart().Z();

     double RX, RY, RZ;
	 trans.GetRotation().GetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, RX, RY, RZ);

	 double pe[6]{X,Y,Z,RX,RY,RZ};
	 double pm[16];
	 Eigen::Matrix4d res;
	 s_pe2pm(pe, res, "123");
	return res;
}


static Eigen::Matrix4d transf2Matrix_02(const gp_Trsf& trans)
{
	Eigen::Matrix4d res;

	res(0, 0) = trans.Value(1, 1); res(0, 1) = trans.Value(1, 2); res(0, 2) = trans.Value(1, 3); res(0, 3) = trans.Value(1, 4);
	res(1, 0) = trans.Value(2, 1); res(1, 1) = trans.Value(2, 2); res(1, 2) = trans.Value(2, 3); res(1, 3) = trans.Value(2, 4);
	res(2, 0) = trans.Value(3, 1); res(2, 1) = trans.Value(3, 2); res(2, 2) = trans.Value(3, 3); res(2, 3) = trans.Value(3, 4);
	res(3, 0) = 0;                 res(3, 1) = 0;                 res(3, 2) = 0;                 res(3, 3) = 1;

	return res;
}

static gp_Trsf Matrix2transf(const Eigen::Matrix4d& matrix)
{
	gp_Trsf res;
	double pe[6];
	s_pm2pe(matrix, pe, "123");

	gp_Trsf trans, rot;
	trans.SetTranslationPart(gp_Vec(pe[0], pe[1], pe[2]));
	
	gp_Quaternion origin_qua;
	origin_qua.SetEulerAngles(gp_EulerSequence::gp_Extrinsic_XYZ, pe[3], pe[4], pe[5]);
	rot.SetRotation(origin_qua);
	res =trans * rot;

	return res;

}







}
#endif // GENERAL_H
