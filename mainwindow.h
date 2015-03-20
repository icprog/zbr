#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include "kinematics.h"
#include "trajectorydialog.h"
#include "plotterdialog.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    static double radToDeg(double rad);
    static double degToRad(double deg);

private:
    Ui::MainWindow *ui;

    Kinematics kinematics;
    Trajectory *trajectory;

    approachVector aV;
    machineCoordinates mC;
    robotParamsRegional rPRegional;
    robotParamsLocal rPLocal;
    Deltas deltas;

    point3D tcpStart, tcpEnd;
    int trajectoryPointNumber;

    QList<machineCoordinates>* resultCoordinates;

    QMessageBox *wrongTrajectoryMessage;
    TrajectoryDialog *trajectoryDialog;
    PlotterDialog *plotDialog;

    void updateKinematics();
    void updateTrajectory();
    void updateStartTCP();
    void updateEndTCP();


private slots:
    void on_updateButton_clicked();
    void on_calcTrajectoryButton_clicked();
    void on_trajectoryEditFinished(Trajectory* trajectory);

    void on_plotButton_clicked();
};

#endif // MAINWINDOW_H
