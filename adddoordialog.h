#ifndef ADDDOORDIALOG_H
#define ADDDOORDIALOG_H

#include <QDialog>

namespace Ui {
class AddDoorDialog;
}

class AddDoorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AddDoorDialog(QWidget *parent = nullptr);
    ~AddDoorDialog();

private:
    Ui::AddDoorDialog *ui;
};

#endif // ADDDOORDIALOG_H
