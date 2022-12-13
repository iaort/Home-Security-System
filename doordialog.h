#ifndef DOORDIALOG_H
#define DOORDIALOG_H


#include <QDialog>

#include "adddoordialog.h"


namespace Ui {
class DoorDialog;
}

class DoorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DoorDialog(QWidget *parent = nullptr, QString name = "door", QString status ="Closed",QString lock = "Locked");
    ~DoorDialog();




private slots:

    void on_pushButton_BackHome_clicked();

    void on_pushButton_clicked();




    void on_FrontD_OPEN_clicked();

    void on_FrontD_CLOSED_clicked();

    void on_BackD_Open_clicked();

    void on_BackD_Closed_clicked();

    void on_SideD_Open_clicked();

    void on_SideD_Close_clicked();

    void on_GarD_Open_clicked();

    void on_GarD_Close_clicked();

    void on_Edit_clicked();

private:
    Ui::DoorDialog *ui;
    AddDoorDialog *addDoorDialog;
    QString name;
    QString status;
    QString lock;
};

#endif // DOORDIALOG_H
