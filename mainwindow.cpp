#include "mainwindow.h"

#include "ui_mainwindow.h"

#include "trajectory.h"
#include "kinematics.h"
#include "math.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
     trajectory = NULL;
     ui->setupUi(this);
     this->setFixedSize(size());
     ui->plotButton->setEnabled(false);
     ui->horizontalSlider->setEnabled(false);
     ui->animationButton->setEnabled(false);
     wrongTrajectoryMessage = new QMessageBox(this);
     wrongTrajectoryMessage->setInformativeText("Wybrana trajektoria jest niemożliwa do zrealizowania.");
     trajectoryDialog = new TrajectoryDialog(this);
     infoDialog = new InfoDialog(this);
     pixmapXY = new QPixmap(ui->XYView->size());
     pixmapYZ = new QPixmap(ui->YZView->size());
     pixmapXZ = new QPixmap(ui->XZView->size());
     pixmapXY->fill(QColor("white"));
     pixmapYZ->fill(QColor("white"));
     pixmapXZ->fill(QColor("white"));
     painterXY = new QPainter(pixmapXY);
     painterXZ = new QPainter(pixmapXZ);
     painterYZ = new QPainter(pixmapYZ);
     ui->XYView->setPixmap(*pixmapXY);
     ui->YZView->setPixmap(*pixmapYZ);
     ui->XZView->setPixmap(*pixmapXZ);

     timer = new QTimer(this);


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
    connect(ui->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(on_horizontalSlider_valueChanged(int)));
    connect(timer, SIGNAL(timeout()), this, SLOT(on_timer_timeout()));
    /* update kinematics with default values */
    updateKinematics();
}

MainWindow::~MainWindow()
{
    delete ui;
    delete painterXY;
    delete painterXZ;
    delete painterYZ;
    delete pixmapXY;
    delete pixmapYZ;
    delete pixmapXZ;
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
point3D MainWindow::getMaxCoords()
{
    point3D max;
    max.x=0;
    max.y=0;
    max.z=0;
    for(int i=0; i<resultCoordinates->size(); i++)
    {
        if(resultCoordinates->at(i).cartesian.p5.x>max.x)
            max.x=resultCoordinates->at(i).cartesian.p5.x;
        if(resultCoordinates->at(i).cartesian.p5.y>max.y)
            max.y=resultCoordinates->at(i).cartesian.p5.y;
        if(resultCoordinates->at(i).cartesian.p5.z>max.z)
            max.z=resultCoordinates->at(i).cartesian.p5.z;
    }
    return max;
}
void MainWindow::paintXY(int i)
{
    if(resultCoordinates->isEmpty())
        return;
    painterXY->setPen(QPen(QBrush(Qt::black), 2));
    QPointF p0(0,0);
    QPointF p1(resultCoordinates->at(i).cartesian.p1.x,resultCoordinates->at(i).cartesian.p1.y);
    QPointF p1pr(resultCoordinates->at(i).cartesian.p1pr.x,resultCoordinates->at(i).cartesian.p1pr.y);
    QPointF p2(resultCoordinates->at(i).cartesian.p2.x,resultCoordinates->at(i).cartesian.p2.y);
    QPointF p2pr(resultCoordinates->at(i).cartesian.p2pr.x,resultCoordinates->at(i).cartesian.p2pr.y);
    QPointF p3(resultCoordinates->at(i).cartesian.p3.x,resultCoordinates->at(i).cartesian.p3.y);
    QPointF p4(resultCoordinates->at(i).cartesian.p4.x,resultCoordinates->at(i).cartesian.p4.y);
    QPointF p5(resultCoordinates->at(i).cartesian.p5.x,resultCoordinates->at(i).cartesian.p5.y);


    p1*=factor;
    p1pr*=factor;
    p2*=factor;
    p2pr*=factor;
    p3*=factor;
    p4*=factor;
    p5*=factor;

    QPointF offset(pixmapXY->width()/2, pixmapXY->height()/2);
    p0+=offset;
    p1+=offset;
    p1pr+=offset;
    p2+=offset;
    p2pr+=offset;
    p3+=offset;
    p4+=offset;
    p5+=offset;

    p1.setX(pixmapXY->width() - p1.x());
    p1.setY(pixmapXY->height() - p1.y());
    p1pr.setX(pixmapXY->width() - p1pr.x());
    p1pr.setY(pixmapXY->height() - p1pr.y());
    p2.setX(pixmapXY->width() - p2.x());
    p2.setY(pixmapXY->height() - p2.y());
    p2pr.setX(pixmapXY->width() - p2pr.x());
    p2pr.setY(pixmapXY->height() - p2pr.y());
    p3.setX(pixmapXY->width() - p3.x());
    p3.setY(pixmapXY->height() - p3.y());
    p4.setX(pixmapXY->width() - p4.x());
    p4.setY(pixmapXY->height() - p4.y());
    p5.setX(pixmapXY->width() - p5.x());
    p5.setY(pixmapXY->height() - p5.y());

    painterXY->eraseRect(0,0,pixmapXY->width(), pixmapXY->height());
    painterXY->drawLine(p0,p1);
    painterXY->drawLine(p1, p1pr);
    painterXY->drawLine(p1pr, p2pr);
    painterXY->drawLine(p2pr, p2);
    painterXY->drawLine(p2, p3);
    painterXY->drawLine(p3, p4);
    painterXY->drawLine(p4, p5);

    painterXY->setPen(QPen(QBrush(Qt::red), 4));
    painterXY->drawPoint(p5);
    painterXY->setPen(QPen(QBrush(Qt::blue), 4));
    painterXY->drawPoint(p3);
    painterXY->setPen(QPen(QBrush(Qt::magenta), 1));
    for(int i=0; i<trajectory->getTrajectory()->size()-1; i++)
    {
        QPointF a(trajectory->getTrajectory()->at(i).x,trajectory->getTrajectory()->at(i).y);
        QPointF b(trajectory->getTrajectory()->at(i+1).x,trajectory->getTrajectory()->at(i+1).y);
        a*=factor;
        b*=factor;
        a+=offset;
        b+=offset;
        a.setX(pixmapXY->width() - a.x());
        a.setY(pixmapXY->height() - a.y());
        b.setX(pixmapXY->width() - b.x());
        b.setY(pixmapXY->height() - b.y());
        painterXY->drawLine(a,b);
    }
    ui->XYView->setPixmap(*pixmapXY);


}
void MainWindow::paintYZ(int i)
{
    if(resultCoordinates->isEmpty())
        return;
    painterYZ->setPen(QPen(QBrush(Qt::black), 2));
    QPointF p0(0,0);
    QPointF p1(resultCoordinates->at(i).cartesian.p1.y,resultCoordinates->at(i).cartesian.p1.z);
    QPointF p1pr(resultCoordinates->at(i).cartesian.p1pr.y,resultCoordinates->at(i).cartesian.p1pr.z);
    QPointF p2(resultCoordinates->at(i).cartesian.p2.y,resultCoordinates->at(i).cartesian.p2.z);
    QPointF p2pr(resultCoordinates->at(i).cartesian.p2pr.y,resultCoordinates->at(i).cartesian.p2pr.z);
    QPointF p3(resultCoordinates->at(i).cartesian.p3.y,resultCoordinates->at(i).cartesian.p3.z);
    QPointF p4(resultCoordinates->at(i).cartesian.p4.y,resultCoordinates->at(i).cartesian.p4.z);
    QPointF p5(resultCoordinates->at(i).cartesian.p5.y,resultCoordinates->at(i).cartesian.p5.z);


    p1*=factor;
    p1pr*=factor;
    p2*=factor;
    p2pr*=factor;
    p3*=factor;
    p4*=factor;
    p5*=factor;

    QPointF offset(pixmapYZ->width()/2, pixmapYZ->height()/2);
    p0+=offset;
    p1+=offset;
    p1pr+=offset;
    p2+=offset;
    p2pr+=offset;
    p3+=offset;
    p4+=offset;
    p5+=offset;

    p1.setX(pixmapXY->width() - p1.x());
    p1.setY(pixmapXY->height() - p1.y());
    p1pr.setX(pixmapXY->width() - p1pr.x());
    p1pr.setY(pixmapXY->height() - p1pr.y());
    p2.setX(pixmapXY->width() - p2.x());
    p2.setY(pixmapXY->height() - p2.y());
    p2pr.setX(pixmapXY->width() - p2pr.x());
    p2pr.setY(pixmapXY->height() - p2pr.y());
    p3.setX(pixmapXY->width() - p3.x());
    p3.setY(pixmapXY->height() - p3.y());
    p4.setX(pixmapXY->width() - p4.x());
    p4.setY(pixmapXY->height() - p4.y());
    p5.setX(pixmapXY->width() - p5.x());
    p5.setY(pixmapXY->height() - p5.y());

    painterYZ->eraseRect(0,0,pixmapYZ->width(), pixmapYZ->height());
    painterYZ->drawLine(p0,p1);
    painterYZ->drawLine(p1, p1pr);
    painterYZ->drawLine(p1pr, p2pr);
    painterYZ->drawLine(p2pr, p2);
    painterYZ->drawLine(p2, p3);
    painterYZ->drawLine(p3, p4);
    painterYZ->drawLine(p4, p5);
    painterYZ->setPen(QPen(QBrush(Qt::red), 4));
    painterYZ->drawPoint(p5);
    painterYZ->setPen(QPen(QBrush(Qt::blue), 4));
    painterYZ->drawPoint(p3);
    painterYZ->setPen(QPen(QBrush(Qt::gray), 1));
    painterYZ->drawLine(0,pixmapYZ->height()/2,pixmapYZ->width(),pixmapYZ->height()/2);
    painterYZ->setPen(QPen(QBrush(Qt::magenta), 1));
    for(int i=0; i<trajectory->getTrajectory()->size()-1; i++)
    {
        QPointF a(trajectory->getTrajectory()->at(i).y,trajectory->getTrajectory()->at(i).z);
        QPointF b(trajectory->getTrajectory()->at(i+1).y,trajectory->getTrajectory()->at(i+1).z);
        a*=factor;
        b*=factor;
        a+=offset;
        b+=offset;
        a.setX(pixmapXY->width() - a.x());
        a.setY(pixmapXY->height() - a.y());
        b.setX(pixmapXY->width() - b.x());
        b.setY(pixmapXY->height() - b.y());
        painterYZ->drawLine(a,b);
    }
    ui->YZView->setPixmap(*pixmapYZ);
}
void MainWindow::paintXZ(int i)
{
    if(resultCoordinates->isEmpty())
        return;
    painterXZ->setPen(QPen(QBrush(Qt::black), 2));
    QPointF p0(0,0);
    QPointF p1(resultCoordinates->at(i).cartesian.p1.x,resultCoordinates->at(i).cartesian.p1.z);
    QPointF p1pr(resultCoordinates->at(i).cartesian.p1pr.x,resultCoordinates->at(i).cartesian.p1pr.z);
    QPointF p2(resultCoordinates->at(i).cartesian.p2.x,resultCoordinates->at(i).cartesian.p2.z);
    QPointF p2pr(resultCoordinates->at(i).cartesian.p2pr.x,resultCoordinates->at(i).cartesian.p2pr.z);
    QPointF p3(resultCoordinates->at(i).cartesian.p3.x,resultCoordinates->at(i).cartesian.p3.z);
    QPointF p4(resultCoordinates->at(i).cartesian.p4.x,resultCoordinates->at(i).cartesian.p4.z);
    QPointF p5(resultCoordinates->at(i).cartesian.p5.x,resultCoordinates->at(i).cartesian.p5.z);


    p1*=factor;
    p1pr*=factor;
    p2*=factor;
    p2pr*=factor;
    p3*=factor;
    p4*=factor;
    p5*=factor;

    QPointF offset(pixmapYZ->width()/2, pixmapYZ->height()/2);
    p0+=offset;
    p1+=offset;
    p1pr+=offset;
    p2+=offset;
    p2pr+=offset;
    p3+=offset;
    p4+=offset;
    p5+=offset;

    p1.setX(pixmapXZ->width() - p1.x());
    p1.setY(pixmapXZ->height() - p1.y());
    p1pr.setX(pixmapXZ->width() - p1pr.x());
    p1pr.setY(pixmapXZ->height() - p1pr.y());
    p2.setX(pixmapXZ->width() - p2.x());
    p2.setY(pixmapXZ->height() - p2.y());
    p2pr.setX(pixmapXZ->width() - p2pr.x());
    p2pr.setY(pixmapXZ->height() - p2pr.y());
    p3.setX(pixmapXZ->width() - p3.x());
    p3.setY(pixmapXZ->height() - p3.y());
    p4.setX(pixmapXZ->width() - p4.x());
    p4.setY(pixmapXZ->height() - p4.y());
    p5.setX(pixmapXZ->width() - p5.x());
    p5.setY(pixmapXZ->height() - p5.y());

    painterXZ->eraseRect(0,0,pixmapYZ->width(), pixmapYZ->height());
    painterXZ->drawLine(p0,p1);
    painterXZ->drawLine(p1, p1pr);
    painterXZ->drawLine(p1pr, p2pr);
    painterXZ->drawLine(p2pr, p2);
    painterXZ->drawLine(p2, p3);
    painterXZ->drawLine(p3, p4);
    painterXZ->drawLine(p4, p5);
    painterXZ->setPen(QPen(QBrush(Qt::red), 4));
    painterXZ->drawPoint(p5);
    painterXZ->setPen(QPen(QBrush(Qt::blue), 4));
    painterXZ->drawPoint(p3);

    painterXZ->setPen(QPen(QBrush(Qt::gray), 1));
    painterXZ->drawLine(0,pixmapXZ->height()/2,pixmapXZ->width(),pixmapXZ->height()/2);
    painterXZ->setPen(QPen(QBrush(Qt::magenta), 1));
    for(int i=0; i<trajectory->getTrajectory()->size()-1; i++)
    {
        QPointF a(trajectory->getTrajectory()->at(i).x,trajectory->getTrajectory()->at(i).z);
        QPointF b(trajectory->getTrajectory()->at(i+1).x,trajectory->getTrajectory()->at(i+1).z);
        a*=factor;
        b*=factor;
        a+=offset;
        b+=offset;
        a.setX(pixmapXY->width() - a.x());
        a.setY(pixmapXY->height() - a.y());
        b.setX(pixmapXY->width() - b.x());
        b.setY(pixmapXY->height() - b.y());
        painterXZ->drawLine(a,b);
    }
    ui->XZView->setPixmap(*pixmapXZ);
}

void MainWindow::on_updateButton_clicked()
{
    updateKinematics();
}

void MainWindow::on_calcTrajectoryButton_clicked()
{
    ui->animationButton->setEnabled(false);
    ui->horizontalSlider->setEnabled(false);
    ui->plotButton->setEnabled(false);
    trajectoryDialog->reset();
    trajectoryDialog->show();

}

void MainWindow::on_trajectoryEditFinished(Trajectory *trajectory)
{
    this->trajectory = trajectory;
    kinematics.setTrajectory(trajectory);
    resultCoordinates = kinematics.getMachineCoordinates();

    if(resultCoordinates != NULL){
        ui->animationButton->setEnabled(true);
        ui->plotButton->setEnabled(true);
        ui->horizontalSlider->setRange(0,resultCoordinates->size()-1);
        ui->horizontalSlider->setEnabled(true);
        point3D maxP = getMaxCoords();
        maxP.x+=150;
        maxP.y+=150;
        maxP.z+=150;
        factor = ui->XYView->size().width() / std::max(maxP.x, std::max(maxP.z,maxP.y));
        factor /= 2;

    }
}

void MainWindow::on_plotButton_clicked()
{
    plotDialog = new PlotterDialog(this);
    plotDialog->setPlotData(resultCoordinates);
    plotDialog->show();
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    paintXY(value);
    paintYZ(value);
    paintXZ(value);
}

void MainWindow::on_animationButton_clicked()
{
    int interval = (11-ui->animSpeedBox->value())*100;
    timer->start(interval);
    ui->horizontalSlider->setValue(0);
}

void MainWindow::on_timer_timeout()
{
    int val = ui->horizontalSlider->value();
    if(val<resultCoordinates->size()-1)
        ui->horizontalSlider->setValue(++val);
    else
    {
        ui->horizontalSlider->setValue(0);
        timer->stop();
    }
}

void MainWindow::on_infoButton_clicked()
{

    infoDialog->show();
}
