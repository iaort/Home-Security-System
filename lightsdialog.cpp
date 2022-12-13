#include "lightsdialog.h"
#include "ui_lightsdialog.h"
#include "secdialog.h"
#include <QSerialPort>
#include <QSerialPortInfo>

QSerialPort serial;
SecDialog *secDialog;



LightsDialog::LightsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LightsDialog)
{
    ui->setupUi(this);
    serial.setPortName("COM8");
    serial.setBaudRate(QSerialPort::Baud9600);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);
    serial.open(QIODevice::ReadWrite);


}

LightsDialog::~LightsDialog()
{
    delete ui;
    serial.close();
}



void LightsDialog::on_pushButton_BackHome_clicked()
{
    serial.close();
    hide();
    secDialog = new SecDialog(this);
    secDialog->show();
}


void LightsDialog::on_pushButton_clicked()
{
    hide();
    addLightDialog = new AddLightDialog(this);
    addLightDialog->show();
}


void LightsDialog::on_GarageL_ON_clicked()
{
    serial.write("q");
}


void LightsDialog::on_GarageL_OFF_clicked()
{
    serial.write("w");
}




void LightsDialog::on_Edit_clicked()
{

    serial.setPortName("COM8");
    serial.setBaudRate(QSerialPort::Baud9600);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);
    serial.open(QIODevice::ReadWrite);
}


void LightsDialog::on_FrontL_ON_clicked()
{
    serial.write("u");
    hide();
//    serial.close();
    secDialog = new SecDialog(this);
    secDialog->ui->label_25->setText(" ON");
    //secDialog->update();
    secDialog->show();
}



void LightsDialog::on_FrontL_OFF_clicked()
{
    serial.write("i");
    hide();
    secDialog = new SecDialog(this);
    secDialog->ui->label_25->setText(" OFF");
    //secDialog->update();
    secDialog->show();
}


void LightsDialog::on_BackL_ON_clicked()
{
    serial.write("e");
}


void LightsDialog::on_BackL_OFF_clicked()
{
    serial.write("r");
}


void LightsDialog::on_SideL_ON_clicked()
{
    serial.write("t");
}


void LightsDialog::on_SideL_OFF_clicked()
{
    serial.write("y");
}

