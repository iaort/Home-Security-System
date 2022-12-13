#include "doordialog.h"
#include "ui_doordialog.h"
#include "ui_secdialog.h"
#include "secdialog.h"
#include <QSerialPort>
#include <QSerialPortInfo>

QSerialPort ser;
SecDialog *secDialog1;

DoorDialog::DoorDialog(QWidget *parent, QString DoorName, QString DoorStatus, QString DoorLock) :
    QDialog(parent),
    ui(new Ui::DoorDialog),
    name(DoorName),
    status(DoorStatus),
    lock(DoorLock)

{
    ui->setupUi(this);
    ser.setPortName("COM8");
    ser.setBaudRate(QSerialPort::Baud9600);
    ser.setDataBits(QSerialPort::Data8);
    ser.setParity(QSerialPort::NoParity);
    ser.setStopBits(QSerialPort::OneStop);
    ser.setFlowControl(QSerialPort::NoFlowControl);
    ser.open(QIODevice::ReadWrite);

}


DoorDialog::~DoorDialog()
{
    delete ui;
    ser.close();
}




void DoorDialog::on_pushButton_BackHome_clicked()
{
    ser.close();
    hide();
    secDialog1 = new SecDialog(this);
    secDialog1->show();

}






void DoorDialog::on_pushButton_clicked()
{
    hide();
    addDoorDialog = new AddDoorDialog(this);
    addDoorDialog->show();
}






void DoorDialog::on_FrontD_OPEN_clicked()
{
    ser.write("o");
//    ser.close();
    secDialog1 = new SecDialog(this);
    secDialog1->ui->label_FrontDoorStatus->setText(" OPENED");
    hide();
    secDialog1->show();
}



void DoorDialog::on_FrontD_CLOSED_clicked()
{
    ser.write("p");
    secDialog1 = new SecDialog(this);
    secDialog1->ui->label_FrontDoorStatus->setText(" CLOSED");
    hide();
    secDialog1->show();
  }


void DoorDialog::on_BackD_Open_clicked()
{
    ser.write("o");
}


void DoorDialog::on_BackD_Closed_clicked()
{
    ser.write("p");
}


void DoorDialog::on_SideD_Open_clicked()
{
   ser.write("o");
}


void DoorDialog::on_SideD_Close_clicked()
{
    ser.write("p");
}


void DoorDialog::on_GarD_Open_clicked()
{
    ser.write("o");

}




void DoorDialog::on_GarD_Close_clicked()
{
    ser.write("p");
}


void DoorDialog::on_Edit_clicked()
{
    ser.setPortName("COM8");
    ser.setBaudRate(QSerialPort::Baud9600);
    ser.setDataBits(QSerialPort::Data8);
    ser.setParity(QSerialPort::NoParity);
    ser.setStopBits(QSerialPort::OneStop);
    ser.setFlowControl(QSerialPort::NoFlowControl);
    ser.open(QIODevice::ReadWrite);
}

