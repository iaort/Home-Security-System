#include "camera.h"
#include "ui_camera.h"
#include <QCamera>
#include "secdialog.h"
//#include <QCaptureViewfinder>
//#include <QCameraImageCapture>

SecDialog *secDialog3;


Camera::Camera(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Camera)
{
    ui->setupUi(this);
    mCamera = new QCamera(this);


}

Camera::~Camera()
{
    delete ui;
}



void Camera::on_Back_Button_clicked()
{
    hide();
    secDialog3 = new SecDialog(this);
    secDialog3->show();
}


void Camera::on_Cam_Button_clicked()
{
//    mCamera->start();
}

