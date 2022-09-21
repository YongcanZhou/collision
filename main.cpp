#include <QApplication>

#include "./src/visualization/mainwindow.h"
// #include "./src/visualization/dlglogin.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
//    DlgLogin* dlg=new DlgLogin();
//    dlg->exec();
    w.show();
    return a.exec();
}
