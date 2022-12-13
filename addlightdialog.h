#ifndef ADDLIGHTDIALOG_H
#define ADDLIGHTDIALOG_H

#include <QDialog>

namespace Ui {
class AddLightDialog;
}

class AddLightDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AddLightDialog(QWidget *parent = nullptr);
    ~AddLightDialog();

private:
    Ui::AddLightDialog *ui;
};

#endif // ADDLIGHTDIALOG_H
