#ifndef DLGLOGIN_H
#define DLGLOGIN_H

#include <QDialog>
#include <QLabel>
#include <QTimer>

class DlgLogin :public QDialog{

    Q_OBJECT
public:
    explicit DlgLogin(QWidget* parent=nullptr);
private:
    QTimer *timer;
    QLabel *label;
private slots:
    void TimeOut();
};


#endif // DLGLOGIN_H


