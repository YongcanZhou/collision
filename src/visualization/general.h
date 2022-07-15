#ifndef GENERAL_H
#define GENERAL_H
#include <QDebug>
#include<Eigen/Eigen>
#include <src/visualization/occ.h>
#include <src/visualization/button_flat.h>
namespace Ui {

#define deltaAngle 0.2
#define deltaLength 0.01
#define transPos 20
#define PI 3.1415926

static ButtonFlat* createViewBtn(QWidget* parent, const QIcon& icon, const QString& tooltip)
{
    auto btn = new ButtonFlat(parent);
    //    btn->setBackgroundBrush(mayoTheme()->color(Theme::Color::ButtonView3d_Background));
    //    btn->setCheckedBrush(mayoTheme()->color(Theme::Color::ButtonView3d_Checked));
    //    btn->setHoverBrush(mayoTheme()->color(Theme::Color::ButtonView3d_Hover));
    btn->setIcon(icon);
    btn->setIconSize(QSize(18, 18));
    btn->setFixedSize(24, 24);
    btn->setToolTip(tooltip);
    return btn;
}

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
        double thetax=pe[5];
        double thetay=pe[4];
        double thetaz=pe[3];
        Eigen::Vector3d eulerAngle(thetax, thetay, thetaz);//(Z-Y-X，即RPY)Eigen::Vector3d eulerAngle(yaw, pitch, roll)
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
        Eigen::Matrix3d matrix_c;
        matrix_c = rollAngle * pitchAngle*yawAngle;
        matrix_c_n = Rotate2RotatePos(matrix_c, pe[0], pe[1], pe[2]);
    }

    if (kind == "321")
    {
        double thetax=pe[5];
        double thetay=pe[4];
        double thetaz=pe[3];
        Eigen::Vector3d eulerAngle(thetax, thetay, thetaz);//(Z-Y-X，即RPY)Eigen::Vector3d eulerAngle(yaw, pitch, roll)
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
        Eigen::Matrix3d matrix_c;
        matrix_c = yawAngle * pitchAngle*rollAngle;
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

    if(kind=="123"){
        eulerAngle=rotation_matrix.eulerAngles(0,1,2);
    }
    if(kind=="321"){
        eulerAngle=rotation_matrix.eulerAngles(2,1,0);
    }


    pe[0]=matrix(0,3); pe[1]=matrix(1,3); pe[2]=matrix(2,3);
    pe[3]=eulerAngle(0); pe[4]=eulerAngle(1); pe[5]=eulerAngle(2);
}

//Modified D-H
struct robotDH{
    struct jointDH{
        double Angle_alfa_1,Length_a_1,Length_d,Angle_theta;
    };
    jointDH joint01dh,joint02dh,joint03dh,joint04dh,joint05dh,joint06dh;
};

static Eigen::Matrix4d ModifyDHconvertMatrix(const double& Length_ai_1,const double&Angle_afa_1,const double&Length_di1,const double&Angle_theta){
    Eigen::Matrix4d res;
    res(0,0)=std::cos(Angle_theta);                        res(0,1)=-std::sin(Angle_theta);                        res(0,2)=0;                           res(0,3)=Length_ai_1;
    res(1,0)=std::sin(Angle_theta)*std::cos(Angle_afa_1);  res(1,1)=std::cos(Angle_theta)*std::cos(Angle_afa_1);   res(1,2)=-std::sin(Angle_afa_1);      res(1,3)=-Length_di1*std::sin(Angle_afa_1);
    res(2,0)=std::sin(Angle_theta)*std::sin(Angle_afa_1);  res(2,1)=std::cos(Angle_theta)*std::sin(Angle_afa_1);   res(2,2)= std::cos(Angle_afa_1);      res(2,3)= Length_di1*std::cos(Angle_afa_1);
    res(3,0)=0;                                            res(3,1)=0;                                             res(3,2)=0;                           res(3,3)=1;
    return res;
}

static Eigen::Matrix4d StandardDHconvertMatrix(const double& Length_ai_1,const double&Angle_afa_1,const double&Length_di1,const double&Angle_theta){
    Eigen::Matrix4d res;
    res(0,0)=std::cos(Angle_theta);   res(0,1)=-std::sin(Angle_theta)*std::cos(Angle_afa_1);  res(0,2)=std::sin(Angle_theta)*std::sin(Angle_afa_1);     res(0,3)=Length_ai_1*std::cos(Angle_theta);
    res(1,0)=std::sin(Angle_theta);   res(1,1)= std::cos(Angle_theta)*std::cos(Angle_afa_1);  res(1,2)=-std::cos(Angle_theta)*std::sin(Angle_afa_1);    res(1,3)=Length_ai_1*std::sin(Angle_theta);
    res(2,0)=0;                       res(2,1)=std::sin(Angle_afa_1);                         res(2,2)= std::cos(Angle_afa_1);                          res(2,3)= Length_di1;
    res(3,0)=0;                       res(3,1)=0;                                             res(3,2)=0;                                               res(3,3)=1;
    return res;
}

static Eigen::Matrix4d KUKARobotForwardCal(const double* thetas){
    Eigen::Matrix4d res;
    robotDH rob;
    rob.joint01dh.Length_d=0.089159;    rob.joint01dh.Length_a_1=0;   rob.joint01dh.Angle_alfa_1=PI/2;     rob.joint01dh.Angle_theta=0;
    rob.joint02dh.Length_d=0;    rob.joint02dh.Length_a_1=-0.425;   rob.joint02dh.Angle_alfa_1=0;         rob.joint02dh.Angle_theta=0;
    rob.joint03dh.Length_d=0;    rob.joint03dh.Length_a_1=-0.39225;    rob.joint03dh.Angle_alfa_1=0;      rob.joint03dh.Angle_theta=0;
    rob.joint04dh.Length_d=0.10915;  rob.joint04dh.Length_a_1=0;     rob.joint04dh.Angle_alfa_1=PI/2;      rob.joint04dh.Angle_theta=0;
    rob.joint05dh.Length_d=0.09465;    rob.joint05dh.Length_a_1=0;     rob.joint05dh.Angle_alfa_1=-PI/2;      rob.joint05dh.Angle_theta=0;
    rob.joint06dh.Length_d=0.0823;  rob.joint06dh.Length_a_1=0;     rob.joint06dh.Angle_alfa_1=0;      rob.joint06dh.Angle_theta=0;

    //    auto res01=ModifyDHconvertMatrix(rob.joint01dh.Length_a_1,rob.joint01dh.Angle_alfa_1,rob.joint01dh.Length_d,rob.joint01dh.Angle_theta+thetas[0]);
    //    auto res02=ModifyDHconvertMatrix(rob.joint02dh.Length_a_1,rob.joint02dh.Angle_alfa_1,rob.joint02dh.Length_d,rob.joint02dh.Angle_theta+thetas[1]);
    //    auto res03=ModifyDHconvertMatrix(rob.joint03dh.Length_a_1,rob.joint03dh.Angle_alfa_1,rob.joint03dh.Length_d,rob.joint03dh.Angle_theta+thetas[2]);
    //    auto res04=ModifyDHconvertMatrix(rob.joint04dh.Length_a_1,rob.joint04dh.Angle_alfa_1,rob.joint04dh.Length_d,rob.joint04dh.Angle_theta+thetas[3]);
    //    auto res05=ModifyDHconvertMatrix(rob.joint05dh.Length_a_1,rob.joint05dh.Angle_alfa_1,rob.joint05dh.Length_d,rob.joint05dh.Angle_theta+thetas[4]);
    //    auto res06=ModifyDHconvertMatrix(rob.joint06dh.Length_a_1,rob.joint06dh.Angle_alfa_1,rob.joint06dh.Length_d,rob.joint06dh.Angle_theta+thetas[5]);

    auto res01=StandardDHconvertMatrix(rob.joint01dh.Length_a_1,rob.joint01dh.Angle_alfa_1,rob.joint01dh.Length_d,rob.joint01dh.Angle_theta+thetas[0]);
    auto res02=StandardDHconvertMatrix(rob.joint02dh.Length_a_1,rob.joint02dh.Angle_alfa_1,rob.joint02dh.Length_d,rob.joint02dh.Angle_theta+thetas[1]);
    auto res03=StandardDHconvertMatrix(rob.joint03dh.Length_a_1,rob.joint03dh.Angle_alfa_1,rob.joint03dh.Length_d,rob.joint03dh.Angle_theta+thetas[2]);
    auto res04=StandardDHconvertMatrix(rob.joint04dh.Length_a_1,rob.joint04dh.Angle_alfa_1,rob.joint04dh.Length_d,rob.joint04dh.Angle_theta+thetas[3]);
    auto res05=StandardDHconvertMatrix(rob.joint05dh.Length_a_1,rob.joint05dh.Angle_alfa_1,rob.joint05dh.Length_d,rob.joint05dh.Angle_theta+thetas[4]);
    auto res06=StandardDHconvertMatrix(rob.joint06dh.Length_a_1,rob.joint06dh.Angle_alfa_1,rob.joint06dh.Length_d,rob.joint06dh.Angle_theta+thetas[5]);

    res=res01*res02*res03*res04*res05*res06;
    qDebugMatrix4d(res,"res");
    return res;
}

static Eigen::Matrix4d UR5RobotForwardCal(const double* thetas){
    double d[6 + 1] = { 0, 0.089159,0,0,0.10915,0.09465,0.0823 };//第0个不用
    double a[6] = { 0,-0.42500,-0.39225,0,0,0 };
    double alpha[6] = { 1.570796, 0, 0, 1.570796, -1.570796, 0 };
    double theta_input[7]{0,thetas[0],thetas[1],thetas[2],thetas[3],thetas[4],thetas[5]};
    Eigen::Matrix4d T[6 + 1];//为了和theta对应，0不用
    for (int i = 1; i <= 6; i++)
    {
        T[i](0, 0) = cos(theta_input[i]);
        T[i](0, 1) = -sin(theta_input[i]) * cos(alpha[i - 1]);
        T[i](0, 2) = sin(theta_input[i]) * sin(alpha[i - 1]);
        T[i](0, 3) = a[i - 1] * cos(theta_input[i]);
        T[i](1, 0) = sin(theta_input[i]);
        T[i](1, 1) = cos(theta_input[i]) * cos(alpha[i - 1]);
        T[i](1, 2) = -cos(theta_input[i]) * sin(alpha[i - 1]);
        T[i](1, 3) = a[i - 1] * sin(theta_input[i]);
        T[i](2, 0) = 0;
        T[i](2, 1) = sin(alpha[i - 1]);
        T[i](2, 2) = cos(alpha[i - 1]);
        T[i](2, 3) = d[i];
        T[i](3, 0) = 0;
        T[i](3, 1) = 0;
        T[i](3, 2) = 0;
        T[i](3, 3) = 1;
    }
    Eigen::Matrix4d T0 = T[1] * T[2] * T[3] * T[4] * T[5] * T[6];
    //    qDebug() << "检验得：" << T0(0, 0) << " " << T0(0, 1) << " " << T0(0, 2) << " px = " << T0(0, 3) << endl;
    //    qDebug()  << "检验得：" << T0(1, 0) << " " << T0(1, 1) << " " << T0(1, 2) << " py = " << T0(1, 3) << endl;
    //    qDebug()  << "检验得：" << T0(2, 0) << " " << T0(2, 1) << " " << T0(2, 2) << " pz = " << T0(2, 3) << endl;
    //    qDebug()  << "检验得：" << T0(0, 3) << " " << T0(1, 3) << " " << T0(2, 3) << endl;
    return T0;
}

static void UR5RobotInverseCal(const Eigen::Matrix4d& input,double* output){

    double d[6 + 1] = { 0, 0.089159,0,0,0.10915,0.09465,0.0823 };//第0个不用
    double a[6] = { 0,-0.42500,-0.39225,0,0,0 };
    double alpha[6] = { 1.570796, 0, 0, 1.570796, -1.570796, 0 };
    double theta[8 + 1][6 + 1];//八组解，每组解六个角，第0个都不用

    double pe[6]{0.0};
    Ui::s_pm2pe(input,pe,"321");
    double x{pe[0]}, y{pe[1]}, z{pe[2]}, RX{pe[5]}, RY{pe[4]}, RZ{pe[3]};

    //1.旋转向量转旋转矩阵，即求T06中的r
    Eigen::Vector3d v(RZ, RY, RX);
    double t_alpha = v.norm();//求模
    v.normalize();//标准化
    Eigen::AngleAxisd rv(t_alpha, v);//旋转向量
    Eigen::Matrix3d rm;
    rm = rv.matrix();

    rm=input.block(0,0,3,3);

    //2.求解
    double A, B, C, D, E, F, G, M, N;//用大写字母替代常数

    //注意，由于数组下标从0开始的问题，矩阵第一行第一列的元素是(0,0)
    //theta1
    A = rm(0, 2) * d[6] - x;
    B = rm(1, 2) * d[6] - y;
    C = d[4];
    //第一个解，赋给一到四组
    theta[1][1] = atan2(B, A) - atan2(C, sqrt(A * A + B * B - C * C));
    theta[2][1] = theta[1][1];
    theta[3][1] = theta[1][1];
    theta[4][1] = theta[1][1];
    //第二个解，赋给五到八组
    theta[5][1] = atan2(B, A) - atan2(C, -sqrt(A * A + B * B - C * C));
    theta[6][1] = theta[5][1];
    theta[7][1] = theta[5][1];
    theta[8][1] = theta[5][1];

    //theta5
    //由theta[1][1]产生的第一个解，赋给一到二组
    A = sin(theta[1][1]) * rm(0, 2) - cos(theta[1][1]) * rm(1, 2);
    theta[1][5] = atan2(sqrt(1 - A * A), A);
    theta[2][5] = theta[1][5];
    //由theta[1][1]产生的第二个解，赋给三到四组
    theta[3][5] = atan2(-sqrt(1 - A * A), A);
    theta[4][5] = theta[3][5];
    //由theta[5][1]产生的第一个解，赋给五到六组
    A = sin(theta[5][1]) * rm(0, 2) - cos(theta[5][1]) * rm(1, 2);
    theta[5][5] = atan2(sqrt(1 - A * A), A);
    theta[6][5] = theta[5][5];
    //由theta[5][1]产生的第二个解，赋给七到八组
    theta[7][5] = atan2(-sqrt(1 - A * A), A);
    theta[8][5] = theta[7][5];

    //theta6
    for (int i = 1; i <= 8; i++)
    {
        A = (-sin(theta[i][1]) * rm(0, 1) + cos(theta[i][1]) * rm(1, 1)) / theta[i][5];
        B = (sin(theta[i][1]) * rm(0, 0) - cos(theta[i][1]) * rm(1, 0)) / theta[i][5];
        theta[i][6] = atan2(A, B);
    }

    //theta2、theta3、theta4
    for (int i = 1; i <= 8; i = i + 2)
    {
        //先算theta2+theta3+theta4
        double theta234[8 + 1];
        A = rm(2, 2) / sin(theta[i][5]);
        B = (cos(theta[i][1]) * rm(0, 2) + sin(theta[i][1]) * rm(1, 2)) / sin(theta[i][5]);
        theta234[i] = atan2(-A, -B) - EIGEN_PI;
        theta234[i + 1] = theta234[i];

        //消去theta2+theta3，计算theta2
        A = -cos(theta234[i]) * sin(theta[i][5]) * d[6] + sin(theta234[i]) * d[5];
        B = -sin(theta234[i]) * sin(theta[i][5]) * d[6] - cos(theta234[i]) * d[5];
        C = cos(theta[i][1]) * x + sin(theta[i][1]) * y;
        D = z - d[1];
        M = C - A;
        N = D - B;
        E = -2 * N * a[2];
        F = 2 * M * a[2];
        G = M * M + N * N + a[2] * a[2] - a[3] * a[3];
        theta[i][2] = atan2(F, E) - atan2(G, sqrt(E * E + F * F - G * G));
        theta[i + 1][2] = atan2(F, E) - atan2(G, -sqrt(E * E + F * F - G * G));

        //用theta2反求theta2+theta3
        double theta23[8 + 1];
        theta23[i] = atan2((N - sin(theta[i][2]) * a[2]) / a[3], (M - cos(theta[i][2]) * a[2]) / a[3]);
        theta23[i + 1] = atan2((N - sin(theta[i + 1][2]) * a[2]) / a[3], (M - cos(theta[i + 1][2]) * a[2]) / a[3]);

        //theta3
        theta[i][3] = theta23[i] - theta[i][2];
        theta[i + 1][3] = theta23[i + 1] - theta[i + 1][2];

        //theta4
        theta[i][4] = theta234[i] - theta23[i];
        theta[i + 1][4] = theta234[i + 1] - theta23[i + 1];
    }

    //输出并检验
    for (int i = 1; i <= 8; i++)
    {
        qDebug() << "第" << i << "组解：" << endl;
        for (int j = 1; j <= 6; j++)
            qDebug()<< "theta" << j << "=" << theta[i][j] * 180 / PI << "  ";
    }
    output[0]=theta[1][1];
    output[1]=theta[1][2];
    output[2]=theta[1][3];
    output[3]=theta[1][4];
    output[4]=theta[1][5];
    output[5]=theta[1][6];


    //    output[0]=0.5;
    //    output[1]=0.5;
    //    output[2]=0.5;
    //    output[3]=0.5;
    //    output[4]=0.5;
    //    output[5]=0.5;
}


static void PILimit(double& data){
    if(data>PI){
        data=data-2*PI;
    }
    if(data<-PI){
        data=data+2*PI;
    }
}

static int UR5RobotInverseCal02(const Eigen::Matrix4d& input,double* output){

    double d1 = 89.159, d2 = 0, d3 = 0, d4 = 109.15, d5 = 94.65, d6 = 82.3;//ur10的D-H参数
    double a1 = 0, a2 = -425, a3 = -392.25;

    //末端位姿矩阵元素
    double nx = input(0,0),  ny =input(1,0),  nz = input(2,0);
    double ox = input(0,1),  oy = input(1,1), oz = input(2,1);
    double ax = input(0,2),  ay = input(1,2), az = input(2,2);
    double px = input(0,3)*1000,  py = input(1,3)*1000, pz = input(2,3)*1000;


    py = py - ay*d6;
    px = px - ax*d6;
    pz = pz - az*d6;
    double angle[8][6] = { 0.0 };////初始的8组解

    if (px*px + py*py - d4*d4 < 0) {
        qDebug() << "不在求解空间，无法求解";
        return 0;
    }
    else {
        //计算theta1(两组解)
        double theta1_positive = atan2(py, px) -atan2(-d4, sqrt(px*px + py*py - d4*d4));
        double theta1_negtive = atan2(py, px) -atan2(-d4, -sqrt(px*px + py*py - d4*d4));

        //计算theta5（四组解）
        double theta5_positive1 = atan2(pow((pow(sin(theta1_positive)*nx - cos(theta1_positive)*ny ,2) +pow( sin(theta1_positive)*ox-cos(theta1_positive)*oy  ,2)   ),0.5),sin(theta1_positive)*ax-cos(theta1_positive)*ay  );
        double theta5_negtive1 = atan2(-pow((pow(sin(theta1_positive)*nx - cos(theta1_positive)*ny, 2) + pow(sin(theta1_positive)*ox - cos(theta1_positive)*oy, 2)), 0.5), sin(theta1_positive)*ax - cos(theta1_positive)*ay);
        double theta5_positive2 = atan2(pow((pow(sin(theta1_negtive)*nx - cos(theta1_negtive)*ny, 2) + pow(sin(theta1_negtive)*ox - cos(theta1_negtive)*oy, 2)), 0.5), sin(theta1_negtive)*ax - cos(theta1_negtive)*ay);
        double theta5_negtive2 = atan2(-pow((pow(sin(theta1_negtive)*nx - cos(theta1_negtive)*ny, 2) + pow(sin(theta1_negtive)*ox - cos(theta1_negtive)*oy, 2)), 0.5), sin(theta1_negtive)*ax - cos(theta1_negtive)*ay);
        if (abs(sin(theta5_positive1)) > 0)
        {
            double mm_1 = (nx*sin(theta1_positive) - ny*cos(theta1_positive))/ sin(theta5_positive1);
            double nn_1 = -(ox*sin(theta1_positive) - oy*cos(theta1_positive))/ sin(theta5_positive1);
            double theta6_1 = atan2(nn_1, mm_1);

            double theta234_1 = atan2(-az/ sin(theta5_positive1),-(cos(theta1_positive)*ax+sin(theta1_positive)*ay)/ sin(theta5_positive1));
            double B1_1=cos(theta1_positive)*px+sin(theta1_positive)*py-d5*sin(theta234_1);
            double B2_1 = pz - d1 + d5*cos(theta234_1);
            double C_1 = (a2*a2 - a3*a3 + B1_1*B1_1 + B2_1*B2_1)/(2*a2);

            if ((B1_1*B1_1 + B2_1*B2_1 - C_1*C_1) >= 0) {
                double theta2_1 = -atan2(B1_1, B2_1) + atan2(C_1, pow(B2_1*B2_1 + B1_1*B1_1 - C_1*C_1, 0.5));
                double theta2_2 = -atan2(B1_1, B2_1) + atan2(C_1, -pow(B2_1*B2_1 + B1_1*B1_1 - C_1*C_1, 0.5));

                double theta23_1 = atan2((B2_1 - a2*sin(theta2_1)) / a3, (B1_1 - a2*cos(theta2_1)) / a3);
                double theta23_2 = atan2((B2_1 - a2*sin(theta2_2)) / a3, (B1_1 - a2*cos(theta2_2)) / a3);

                double theta3_1 = theta23_1 - theta2_1;
                double theta3_2 = theta23_2 - theta2_2;

                double theta4_1 = theta234_1 - theta2_1 - theta3_1;
                double theta4_2 = theta234_1 - theta2_2 - theta3_2;

                angle[0][0] = theta1_positive; angle[0][1] = theta2_1; angle[0][2] = theta3_1; angle[0][3] = theta4_1; angle[0][4] = theta5_positive1; angle[0][5] = theta6_1;
                angle[1][0] = theta1_positive; angle[1][1] = theta2_2; angle[1][2] = theta3_2; angle[1][3] = theta4_2; angle[1][4] = theta5_positive1; angle[1][5] = theta6_1;
            }
            else {
                qDebug() << "不在求解范围内1" << endl;
            }
        }
        if (abs(sin(theta5_negtive1)) > 0) {
            double mm_1 = (nx*sin(theta1_positive) - ny*cos(theta1_positive))/ sin(theta5_negtive1);
            double nn_1 = -(ox*sin(theta1_positive) - oy*cos(theta1_positive))/ sin(theta5_negtive1);
            double theta6_2 = atan2(nn_1, mm_1);

            double theta234_2 = atan2(-az / sin(theta5_negtive1), -(cos(theta1_positive)*ax + sin(theta1_positive)*ay) / sin(theta5_negtive1));
            double B1_2 = cos(theta1_positive)*px + sin(theta1_positive)*py - d5*sin(theta234_2);
            double B2_2 = pz - d1 + d5*cos(theta234_2);
            double C_2 = (a2*a2 - a3*a3 + B1_2*B1_2 + B2_2*B2_2)/(2*a2) ;

            if ((B1_2*B1_2 + B2_2*B2_2 - C_2*C_2) >= 0) {
                double theta2_3 = -atan2(B1_2, B2_2) + atan2(C_2, pow(B2_2*B2_2 + B1_2*B1_2 - C_2*C_2, 0.5));
                double theta2_4 = -atan2(B1_2, B2_2) + atan2(C_2, -pow(B2_2*B2_2 + B1_2*B1_2 - C_2*C_2, 0.5));

                double theta23_3 = atan2((B2_2 - a2*sin(theta2_3)) / a3, (B1_2 - a2*cos(theta2_3)) / a3);
                double theta23_4 = atan2((B2_2 - a2*sin(theta2_4)) / a3, (B1_2 - a2*cos(theta2_4)) / a3);

                double theta3_3 = theta23_3 - theta2_3;
                double theta3_4 = theta23_4 - theta2_4;

                double theta4_3 = theta234_2 - theta2_3 - theta3_3;
                double theta4_4 = theta234_2 - theta2_4 - theta3_4;

                angle[2][0] = theta1_positive; angle[2][1] = theta2_3; angle[2][2] = theta3_3; angle[2][3] = theta4_3; angle[2][4] = theta5_negtive1; angle[2][5] = theta6_2;
                angle[3][0] = theta1_positive; angle[3][1] = theta2_4; angle[3][2] = theta3_4; angle[3][3] = theta4_4; angle[3][4] = theta5_negtive1; angle[3][5] = theta6_2;
            }
            else {
                qDebug() << "不在求解范围内2" << endl;
            }
        }
        if (abs(sin(theta5_positive2)) > 0) {
            double mm_2 = (nx*sin(theta1_negtive) - ny*cos(theta1_negtive))/ sin(theta5_positive2);
            double nn_2 = (-(ox*sin(theta1_negtive) - oy*cos(theta1_negtive)))/ sin(theta5_positive2);
            double theta6_3 = atan2(nn_2, mm_2);

            double theta234_3 = atan2(-az / sin(theta5_positive2), -(cos(theta1_negtive)*ax + sin(theta1_negtive)*ay) / sin(theta5_positive2));
            double B1_3 = cos(theta1_negtive)*px + sin(theta1_negtive)*py - d5*sin(theta234_3);
            double B2_3 = pz - d1 + d5*cos(theta234_3);
            double C_3 = (a2*a2 - a3*a3 + B1_3*B1_3 + B2_3*B2_3)/(2*a2);
            if ((B1_3*B1_3 + B2_3*B2_3 - C_3*C_3) >= 0) {
                double theta2_5 = -atan2(B1_3, B2_3) + atan2(C_3, pow(B1_3*B1_3 + B2_3*B2_3 - C_3*C_3, 0.5));
                double theta2_6 = -atan2(B1_3, B2_3) + atan2(C_3, -pow(B2_3*B2_3 + B1_3*B1_3 - C_3*C_3, 0.5));

                double theta23_5 = atan2((B2_3 - a2*sin(theta2_5)) / a3, (B1_3 - a2*cos(theta2_5)) / a3);
                double theta23_6 = atan2((B2_3 - a2*sin(theta2_6)) / a3, (B1_3 - a2*cos(theta2_6)) / a3);

                double theta3_5 = theta23_5 - theta2_5;
                double theta3_6 = theta23_6 - theta2_6;

                double theta4_5 = theta234_3 - theta2_5 - theta3_5;
                double theta4_6 = theta234_3 - theta2_6 - theta3_6;

                angle[4][0] = theta1_negtive; angle[4][1] = theta2_5; angle[4][2] = theta3_5; angle[4][3] = theta4_5; angle[4][4] = theta5_positive2; angle[4][5] = theta6_3;
                angle[5][0] = theta1_negtive; angle[5][1] = theta2_6; angle[5][2] = theta3_6; angle[5][3] = theta4_6; angle[5][4] = theta5_positive2; angle[5][5] = theta6_3;
            }
            else {
                qDebug() << "不在求解范围内3" << endl;
            }

        }
        if (abs(sin(theta5_negtive2)) > 0) {
            double mm_2 = (nx*sin(theta1_negtive) - ny*cos(theta1_negtive))/sin(theta5_negtive2);
            double nn_2 = -(ox*sin(theta1_negtive) - oy*cos(theta1_negtive)) / sin(theta5_negtive2);
            double theta6_4 = atan2(nn_2, mm_2);

            double theta234_4 = atan2(-az / sin(theta5_negtive2), -(cos(theta1_negtive)*ax + sin(theta1_negtive)*ay) / sin(theta5_negtive2));
            double B1_4 = cos(theta1_negtive)*px + sin(theta1_negtive)*py - d5*sin(theta234_4);
            double B2_4 = pz - d1 + d5*cos(theta234_4);
            double C_4 = (a2*a2 - a3*a3 + B1_4*B1_4 + B2_4*B2_4)/(2*a2);
            if ((B1_4*B1_4 + B2_4*B2_4 - C_4*C_4) >= 0) {

                double theta2_7 = -atan2(B1_4, B2_4) + atan2(C_4, pow(B1_4*B1_4 + B2_4*B2_4 - C_4*C_4, 0.5));
                double theta2_8 = -atan2(B1_4, B2_4) + atan2(C_4, -pow(B1_4*B1_4 + B2_4*B2_4 - C_4*C_4, 0.5));

                double theta23_7 = atan2((B2_4 - a2*sin(theta2_7)) / a3, (B1_4 - a2*cos(theta2_7)) / a3);
                double theta23_8 = atan2((B2_4 - a2*sin(theta2_8)) / a3, (B1_4 - a2*cos(theta2_8)) / a3);

                double theta3_7 = theta23_7 - theta2_7;
                double theta3_8 = theta23_8 - theta2_8;

                double theta4_7 = theta234_4 - theta2_7 - theta3_7;
                double theta4_8 = theta234_4 - theta2_8 - theta3_8;
                angle[6][0] = theta1_negtive; angle[6][1] = theta2_7; angle[6][2] = theta3_7; angle[6][3] = theta4_7; angle[6][4] = theta5_negtive2; angle[6][5] = theta6_4;
                angle[7][0] = theta1_negtive; angle[7][1] = theta2_8; angle[7][2] = theta3_8; angle[7][3] = theta4_8; angle[7][4] = theta5_negtive2; angle[7][5] = theta6_4;
            }
            else {
                qDebug() << "不在求解范围内4" << endl;
            }
        }




//        qDebug() << "初始8组解：" << endl;
//        for (int i = 0; i < 8; i++) {
//            for (int j = 0; j < 6; j++) {
//                qDebug() << angle[i][j] * 180 / 3.14159 << ", ";
//            }
//            qDebug() << endl;
//        }




        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 6; j++) {
                PILimit(angle[i][j]);
            }
        }


        output[0]=angle[0][0];
        output[1]=angle[0][1];
        output[2]=angle[0][2];
        output[3]=angle[0][3];
        output[4]=angle[0][4];
        output[5]=angle[0][5];
    }

}


struct EularCoor{
    double x,y,z,rx,ry,rz;
};

}

#endif // GENERAL_H
