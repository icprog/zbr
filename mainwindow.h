#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include "kinematics.h"
#include "trajectorydialog.h"
#include "plotterdialog.h"
#include "infodialog.h"

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
    InfoDialog *infoDialog;

    QPainter *painterXY;
    QPainter *painterYZ;
    QPainter *painterXZ;
    QPixmap *pixmapXY;
    QPixmap *pixmapYZ;
    QPixmap *pixmapXZ;

    float factor;

    QTimer *timer;

    void updateKinematics();
    void updateTrajectory();
    void updateStartTCP();
    void updateEndTCP();
    point3D getMaxCoords();

    void paintXY(int i);
    void paintYZ(int i);
    void paintXZ(int i);

private slots:
    void on_updateButton_clicked();
    void on_calcTrajectoryButton_clicked();
    void on_trajectoryEditFinished(Trajectory* trajectory);
    void on_timer_timeout();

    void on_plotButton_clicked();
    void on_horizontalSlider_valueChanged(int value);
    void on_animationButton_clicked();
    void on_infoButton_clicked();
};

#endif // MAINWINDOW_H
