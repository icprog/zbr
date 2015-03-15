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
    approachVector aV;
    machineCoordinates mC;
    robotParamsRegional rPRegional;
    robotParamsLocal rPLocal;
    Deltas deltas;

    void updateKinematics();
    double radToDeg(double rad);
    double degToRad(double deg);

private slots:
    void on_updateButton_clicked();

};

#endif // MAINWINDOW_H
