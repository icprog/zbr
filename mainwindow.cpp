#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "trajectory.h"
#include "kinematics.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}
