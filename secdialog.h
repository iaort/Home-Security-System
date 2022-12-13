#ifndef SECDIALOG_H
#define SECDIALOG_H

#include <QDialog>
#include "windowdialog.h"
#include "lightsdialog.h"
#include "doordialog.h"
#include "ui_secdialog.h"
#include "camera.h"



namespace Ui {
class SecDialog;
}

class SecDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SecDialog(QWidget *parent = nullptr);
     Ui::SecDialog *ui;
    ~SecDialog();





private slots:
    void on_pushButton_Doors_clicked();

    void on_pushButtonWindows_clicked();

    void on_pushButton_Lights_clicked();









    void on_AC_ON_clicked();

    void on_AC_OFF_clicked();



    void on_RECONNECT_clicked();

    void on_CamStart_Button_clicked();


protected:
//    Ui::SecDialog *ui;

private:
//    Ui::SecDialog *ui;
    WindowDialog *windowDialog;
    DoorDialog *doorDialog;
    LightsDialog *lightsDialog;
    QCamera *mCamera;
    Camera *camera;

};

#endif // SECDIALOG_H
