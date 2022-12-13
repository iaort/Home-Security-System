#include "adddoordialog.h"
#include "ui_adddoordialog.h"

AddDoorDialog::AddDoorDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddDoorDialog)
{
    ui->setupUi(this);
}

AddDoorDialog::~AddDoorDialog()
{
    delete ui;
}
