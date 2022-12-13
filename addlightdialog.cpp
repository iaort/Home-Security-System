#include "addlightdialog.h"
#include "ui_addlightdialog.h"

AddLightDialog::AddLightDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddLightDialog)
{
    ui->setupUi(this);
}

AddLightDialog::~AddLightDialog()
{
    delete ui;
}
