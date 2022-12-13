#ifndef ADDWINDOWDIALOG_H
#define ADDWINDOWDIALOG_H

#include <QDialog>

namespace Ui {
class AddWindowDialog;
}

class AddWindowDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AddWindowDialog(QWidget *parent = nullptr);
    ~AddWindowDialog();

private:
    Ui::AddWindowDialog *ui;
};

#endif // ADDWINDOWDIALOG_H
