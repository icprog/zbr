#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "trajectory.h"
#include "kinematics.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
     trajectory = NULL;
     ui->setupUi(this);
     wrongTrajectoryMessage = new QMessageBox(this);
     wrongTrajectoryMessage->setInformativeText("Wybrana trajektoria jest niemożliwa do zrealizowania.");
    /* set up validators */
    QDoubleValidator* l1Validator = new QDoubleValidator(0, 10000,10,ui->l1LineEdit);
    QDoubleValidator* l2Validator = new QDoubleValidator(0, 10000,10,ui->l2LineEdit);
    QDoubleValidator* l3Validator = new QDoubleValidator(0, 10000,10,ui->l3LineEdit);
    QDoubleValidator* l4Validator = new QDoubleValidator(0, 10000,10,ui->l4LineEdit);
    QDoubleValidator* l5Validator = new QDoubleValidator(0, 10000,10,ui->l5LineEdit);
    QDoubleValidator* l6Validator = new QDoubleValidator(0, 10000,10,ui->l6LineEdit);
    QDoubleValidator* dValidator = new QDoubleValidator(0, 10000,10,ui->dLineEdit);
    QDoubleValidator* eValidator = new QDoubleValidator(0, 10000,10,ui->eLineEdit);

    QIntValidator* xpValidator = new QIntValidator(-10000,10000,ui->xpLineEdit);
    QIntValidator* ypValidator = new QIntValidator(-10000,10000,ui->ypLineEdit);
    QIntValidator* zpValidator = new QIntValidator(-10000,10000,ui->zpLineEdit);
    QIntValidator* xkValidator = new QIntValidator(-10000,10000,ui->xkLineEdit);
    QIntValidator* ykValidator = new QIntValidator(-10000,10000,ui->ykLineEdit);
    QIntValidator* zkValidator = new QIntValidator(-10000,10000,ui->zkLineEdit);
    QIntValidator* trajectoryPtsValidator = new QIntValidator(0, 1000, ui->trajectoryPtsLineEdit);


    ui->l1LineEdit->setValidator(l1Validator);
    ui->l2LineEdit->setValidator(l2Validator);
    ui->l3LineEdit->setValidator(l3Validator);
    ui->l4LineEdit->setValidator(l4Validator);
    ui->l5LineEdit->setValidator(l5Validator);
    ui->l6LineEdit->setValidator(l6Validator);
    ui->dLineEdit->setValidator(dValidator);
    ui->eLineEdit->setValidator(eValidator);


    ui->xpLineEdit->setValidator(xpValidator);
    ui->ypLineEdit->setValidator(ypValidator);
    ui->zpLineEdit->setValidator(zpValidator);
    ui->xkLineEdit->setValidator(xkValidator);
    ui->ykLineEdit->setValidator(ykValidator);
    ui->zkLineEdit->setValidator(zkValidator);
    ui->trajectoryPtsLineEdit->setValidator(trajectoryPtsValidator);

    /* set up signals'n'slots */
    connect(ui->updateButton, SIGNAL(clicked()), this, SLOT(on_updateButton_clicked()));
    connect(ui->calcTrajectoryButton, SIGNAL(clicked()),this, SLOT(on_calcTrajectoryButton_clicked()));
    connect(&kinematics, SIGNAL(wrongTCP()), wrongTrajectoryMessage, SLOT(exec()));
    /* update kinematics with default values */
    updateKinematics();
    trajectoryPointNumber = ui->trajectoryPtsLineEdit->text().toInt();
    updateTrajectory();
    QList<machineCoordinates> *yolo = kinematics.getMachineCoordinates();
}

MainWindow::~MainWindow()
{
    delete ui;
}

double MainWindow::radToDeg(double rad)
{
    return rad * (180/M_PI);
}

double MainWindow::degToRad(double deg)
{
    return deg*M_PI/180;
}

void MainWindow::updateKinematics()
{
    aV.psi = degToRad((ui->psiLineEdit->text()).toDouble());
    aV.theta = degToRad((ui->thetaLineEdit->text()).toDouble());

    rPRegional.l1 = (ui->l1LineEdit->text()).toDouble();
    rPRegional.l2 = (ui->l2LineEdit->text()).toDouble();
    rPRegional.l3 = (ui->l3LineEdit->text()).toDouble();
    rPRegional.d = (ui->dLineEdit->text()).toDouble();
    rPRegional.e = (ui->eLineEdit->text()).toDouble();

    rPLocal.l4 = (ui->l4LineEdit->text()).toDouble();
    rPLocal.l5 = (ui->l5LineEdit->text()).toDouble();
    rPLocal.l6 = (ui->l6LineEdit->text()).toDouble();

    deltas.d1 = ui->d1SpinBox->value();
    deltas.d2 = ui->d2SpinBox->value();
    deltas.d5 = ui->d5SpinBox->value();

    kinematics.setApproachVector(aV);
    kinematics.setDeltas(deltas);
    kinematics.setRobotParamsLocal(rPLocal);
    kinematics.setRobotParamsRegional(rPRegional);
}

void MainWindow::updateTrajectory()
{
    if(trajectory!=NULL)
    {
        delete trajectory;
        trajectory = NULL;
    }
    updateStartTCP();
    updateEndTCP();
    trajectory = new Trajectory(tcpStart,tcpEnd,trajectoryPointNumber);
    kinematics.setTrajectory(trajectory);

}

void MainWindow::updateStartTCP()
{
    tcpStart.x = ui->xpLineEdit->text().toInt();
    tcpStart.y = ui->ypLineEdit->text().toInt();
    tcpStart.z = ui->zpLineEdit->text().toInt();
}

void MainWindow::updateEndTCP()
{
    tcpEnd.x = ui->xkLineEdit->text().toInt();
    tcpEnd.y = ui->ykLineEdit->text().toInt();
    tcpEnd.z = ui->zkLineEdit->text().toInt();
}

void MainWindow::on_updateButton_clicked()
{
    updateKinematics();
}

void MainWindow::on_calcTrajectoryButton_clicked()
{
    updateTrajectory();
}

