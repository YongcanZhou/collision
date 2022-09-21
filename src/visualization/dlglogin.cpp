#include "dlglogin.h"


DlgLogin::DlgLogin(QWidget *parent):QDialog (parent)
{
    label=new QLabel(this);
    label->setGeometry(QRect(225,200,150,65));
    label->setStyleSheet("font-weight:bold;border-width:0px;");
    label->setAlignment(Qt::AlignCenter);
    label->setText(tr("Welcome, please wait..."));

    timer = new QTimer(this);
    timer->setSingleShot(true);
    timer->setInterval(3000);
    connect(timer, SIGNAL(timeout()), this, SLOT(TimeOut()));
    timer->start();
}

void DlgLogin::TimeOut()
{
    this->accept();
}
