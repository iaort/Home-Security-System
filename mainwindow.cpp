#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <string>




using namespace std;




MainWindow::MainWindow(QWidget *parent,QString UserName, QString Password)
    : QMainWindow(parent),
     ui(new Ui::MainWindow),
     user(UserName),
     pass(Password)

{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}




void MainWindow::on_pushButton_clicked()
{
     user = ui->lineEdit->text();
     pass = ui->lineEdit_2->text();

    if ((user == "Sasha" ) && (pass == "12345")) {

       QMessageBox::information(this,"Login", "Username and Password is correct");
       hide();
        secDialog = new SecDialog(this);
        secDialog->show();
    }
    else {
         QMessageBox::warning(this, "Login", "Username and Password is not correct");

    }
}


void MainWindow::on_pushButton_2_clicked()
{



    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this,"Login"," Are you sure you want to close the application?", QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes){
        QApplication::quit();
    }
}




