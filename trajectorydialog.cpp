#include "trajectorydialog.h"
#include "ui_trajectorydialog.h"

TrajectoryDialog::TrajectoryDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TrajectoryDialog)
{
    ui->setupUi(this);
    this->setFixedSize(size());
    this->setWindowTitle("Edycja trajektorii");
    ui->buttonBox->setEnabled(false);
    trajectory = new Trajectory();

    QIntValidator* xpValidator = new QIntValidator(-10000,10000,ui->xpLineEdit);
    QIntValidator* ypValidator = new QIntValidator(-10000,10000,ui->ypLineEdit);
    QIntValidator* zpValidator = new QIntValidator(-10000,10000,ui->zpLineEdit);
    QIntValidator* xkValidator = new QIntValidator(-10000,10000,ui->xkLineEdit);
    QIntValidator* ykValidator = new QIntValidator(-10000,10000,ui->ykLineEdit);
    QIntValidator* zkValidator = new QIntValidator(-10000,10000,ui->zkLineEdit);
    QIntValidator* trajectoryPtsValidator = new QIntValidator(0, 1000, ui->trajectoryPtsLineEdit);

    ui->xpLineEdit->setValidator(xpValidator);
    ui->ypLineEdit->setValidator(ypValidator);
    ui->zpLineEdit->setValidator(zpValidator);
    ui->xkLineEdit->setValidator(xkValidator);
    ui->ykLineEdit->setValidator(ykValidator);
    ui->zkLineEdit->setValidator(zkValidator);
    ui->trajectoryPtsLineEdit->setValidator(trajectoryPtsValidator);
}

TrajectoryDialog::~TrajectoryDialog()
{
    delete ui;
}

void TrajectoryDialog::reset()
{
    ui->xpLineEdit->setEnabled(true);
    ui->ypLineEdit->setEnabled(true);
    ui->zpLineEdit->setEnabled(true);
    ui->buttonBox->setEnabled(false);
    ui->label_22->setText("Punkt końcowy:");
    trajectory->clearTrajectory();
}



void TrajectoryDialog::on_addPointButton_clicked()
{
    point3D start,end;
    int points = ui->trajectoryPtsLineEdit->text().toInt();
    end.x = ui->xkLineEdit->text().toInt();
    end.y = ui->ykLineEdit->text().toInt();
    end.z = ui->zkLineEdit->text().toInt();
    if(trajectory->isEmpty())
    {
        start.x = ui->xpLineEdit->text().toInt();
        start.y = ui->ypLineEdit->text().toInt();
        start.z = ui->zpLineEdit->text().toInt();
        trajectory->addBegin(start);
        ui->xpLineEdit->setEnabled(false);
        ui->ypLineEdit->setEnabled(false);
        ui->zpLineEdit->setEnabled(false);
    }
    trajectory->addPoint(end, points);
    ui->buttonBox->setEnabled(true);
    ui->label_22->setText("Następny punkt:");
}

void TrajectoryDialog::on_buttonBox_accepted()
{
    emit trajectoryEditFinished(trajectory);
}

void TrajectoryDialog::on_buttonBox_rejected()
{
    delete trajectory;
}
