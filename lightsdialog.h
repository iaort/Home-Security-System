#ifndef LIGHTSDIALOG_H
#define LIGHTSDIALOG_H

#include <QDialog>
#include "addlightdialog.h"

namespace Ui {
class LightsDialog;
}

class LightsDialog : public QDialog {

    Q_OBJECT

public:
    explicit LightsDialog(QWidget *parent = nullptr);
    ~LightsDialog();

private slots:
    void on_pushButton_BackHome_clicked();

    void on_pushButton_clicked();

    void on_GarageL_ON_clicked();

    void on_GarageL_OFF_clicked();

    void on_Edit_clicked();

    void on_FrontL_ON_clicked();

    void on_FrontL_OFF_clicked();

    void on_BackL_ON_clicked();

    void on_BackL_OFF_clicked();

    void on_SideL_ON_clicked();

    void on_SideL_OFF_clicked();

private:
    Ui::LightsDialog *ui;
    AddLightDialog *addLightDialog;

};

#endif // LIGHTSDIALOG_H
