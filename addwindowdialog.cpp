#include "addwindowdialog.h"
#include "ui_addwindowdialog.h"

AddWindowDialog::AddWindowDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddWindowDialog)
{
    ui->setupUi(this);
}

AddWindowDialog::~AddWindowDialog()
{
    delete ui;
}
