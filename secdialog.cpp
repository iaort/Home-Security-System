#include "secdialog.h"
#include "ui_secdialog.h"
#include "doordialog.h"
#include <QCamera>
#include <QSerialPort>
#include <QSerialPortInfo>

QSerialPort seri;



SecDialog::SecDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SecDialog)
{
    ui->setupUi(this);
    mCamera = new QCamera(this);
    seri.setPortName("COM8");
    seri.setBaudRate(QSerialPort::Baud9600);
    seri.setDataBits(QSerialPort::Data8);
    seri.setParity(QSerialPort::NoParity);
    seri.setStopBits(QSerialPort::OneStop);
    seri.setFlowControl(QSerialPort::NoFlowControl);
    seri.open(QIODevice::ReadWrite);
}

SecDialog::~SecDialog()
{
    delete ui;
    seri.close();
}






void SecDialog::on_pushButton_Doors_clicked()
{
seri.close();
hide();
doorDialog = new DoorDialog(this);
doorDialog->show();

}


void SecDialog::on_pushButtonWindows_clicked()
{
    seri.close();
    hide();
    windowDialog = new WindowDialog(this);
    windowDialog->show();
}


void SecDialog::on_pushButton_Lights_clicked()
{
    seri.close();
    hide();
    lightsDialog = new LightsDialog(this);
    lightsDialog-> show();
}






void SecDialog::on_AC_ON_clicked()
{
    ui->AC->setText(seri.readAll());


}



void SecDialog::on_AC_OFF_clicked()
{
    ui->AC->setText("AC OFF");
    seri.write("s");
}




void SecDialog::on_RECONNECT_clicked()
{
    seri.setPortName("COM8");
    seri.setBaudRate(QSerialPort::Baud9600);
    seri.setDataBits(QSerialPort::Data8);
    seri.setParity(QSerialPort::NoParity);
    seri.setStopBits(QSerialPort::OneStop);
    seri.setFlowControl(QSerialPort::NoFlowControl);
    seri.open(QIODevice::ReadWrite);
}


void SecDialog::on_CamStart_Button_clicked()
{
      mCamera->start();
      mCamera->cameraDevice();
      mCamera->flashMode();
}

