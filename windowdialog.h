#ifndef WINDOWDIALOG_H
#define WINDOWDIALOG_H

#include <QDialog>

#include "addwindowdialog.h"

namespace Ui {
class WindowDialog;
}

class WindowDialog : public QDialog
{
    Q_OBJECT

public:
    explicit WindowDialog(QWidget *parent = nullptr);
    ~WindowDialog();

private slots:
    void on_pushButton_BackHome_clicked();

    void on_pushButton_clicked();

    void on_FrontW_Open_clicked();

    void on_FrontW_Close_clicked();

    void on_BackW_Open_clicked();

    void on_BackW_Close_clicked();

    void on_SideW_Open_clicked();

    void on_SideW_Close_clicked();

    void on_Edit_clicked();

private:
    Ui::WindowDialog *ui;
    AddWindowDialog *addWindowDialog;
};

#endif // WINDOWDIALOG_H
