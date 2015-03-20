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
     this->setFixedSize(size());
     wrongTrajectoryMessage = new QMessageBox(this);
     wrongTrajectoryMessage->setInformativeText("Wybrana trajektoria jest niemożliwa do zrealizowania.");
     trajectoryDialog = new TrajectoryDialog(this);

    /* set up validators */
    QDoubleValidator* l1Validator = new QDoubleValidator(0, 10000,10,ui->l1LineEdit);
    QDoubleValidator* l2Validator = new QDoubleValidator(0, 10000,10,ui->l2LineEdit);
    QDoubleValidator* l3Validator = new QDoubleValidator(0, 10000,10,ui->l3LineEdit);
    QDoubleValidator* l4Validator = new QDoubleValidator(0, 10000,10,ui->l4LineEdit);
    QDoubleValidator* l5Validator = new QDoubleValidator(0, 10000,10,ui->l5LineEdit);
    QDoubleValidator* l6Validator = new QDoubleValidator(0, 10000,10,ui->l6LineEdit);
    QDoubleValidator* dValidator = new QDoubleValidator(0, 10000,10,ui->dLineEdit);
    QDoubleValidator* eValidator = new QDoubleValidator(0, 10000,10,ui->eLineEdit);




    ui->l1LineEdit->setValidator(l1Validator);
    ui->l2LineEdit->setValidator(l2Validator);
    ui->l3LineEdit->setValidator(l3Validator);
    ui->l4LineEdit->setValidator(l4Validator);
    ui->l5LineEdit->setValidator(l5Validator);
    ui->l6LineEdit->setValidator(l6Validator);
    ui->dLineEdit->setValidator(dValidator);
    ui->eLineEdit->setValidator(eValidator);




    /* set up signals'n'slots */
    connect(ui->updateButton, SIGNAL(clicked()), this, SLOT(on_updateButton_clicked()));
    connect(ui->calcTrajectoryButton, SIGNAL(clicked()),this, SLOT(on_calcTrajectoryButton_clicked()));
    connect(&kinematics, SIGNAL(wrongTCP()), wrongTrajectoryMessage, SLOT(exec()));
    connect(trajectoryDialog, SIGNAL(trajectoryEditFinished(Trajectory*)), this, SLOT(on_trajectoryEditFinished(Trajectory*)));
    /* update kinematics with default values */
    updateKinematics();
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

void MainWindow::on_updateButton_clicked()
{
    updateKinematics();
}

void MainWindow::on_calcTrajectoryButton_clicked()
{

    trajectoryDialog->reset();
    trajectoryDialog->show();

}

void MainWindow::on_trajectoryEditFinished(Trajectory *trajectory)
{
    kinematics.setTrajectory(trajectory);
    resultCoordinates = kinematics.getMachineCoordinates();
}

void MainWindow::on_plotButton_clicked()
{
    plotDialog = new PlotterDialog(this);
    plotDialog->setPlotData(resultCoordinates);
    plotDialog->show();
}
