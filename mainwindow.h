#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "kinematics.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

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

    void updateKinematics();
    void updateTrajectory();
    void updateStartTCP();
    void updateEndTCP();
    double radToDeg(double rad);
    double degToRad(double deg);

private slots:
    void on_updateButton_clicked();
    void on_calcTrajectoryButton_clicked();

};

#endif // MAINWINDOW_H
