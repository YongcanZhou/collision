#include "src/visualization/mainwindow.h"
#include <QApplication>
#include "src/simulator/dynamic_simulator.h"
#include <array>

int main(int argc, char *argv[])
{
//    zyc::InitSimulator();
//    zyc::SimThreadFun();
/*    std::array<double,6*16> link_pm_test;
    zyc::DynamicSimulator(link_pm_test);
    int z{0};
    for(int i=0;i<6;++i){
//        m->partPool().at(i).getPm(link);
        for(int j=0;j<16;++j){
//            link_position[j+i*16] = link[j];
            std::cout<<"num"<<z<<":"<<link_pm_test[j+i*16]<<" ";
            z+=1;
        }
    }*/
//    std::array<double, 16> test = zyc::SimThreadFun();
//    std::cout<<"test"<<test[3]<<std::enidl;
//    return 0;

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
