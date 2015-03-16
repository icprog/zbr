#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "typedefs.h"
#include "trajectory.h"
#include "qmath.h"
#include <QObject>


class Kinematics : public QObject
{
    Q_OBJECT
public:
    Kinematics();
    Kinematics(robotParamsLocal local, robotParamsRegional regional, Deltas deltas, approachVector vector);
    ~Kinematics();

    void setRobotParamsLocal(robotParamsLocal local);
    void setRobotParamsRegional(robotParamsRegional regional);
    void setDeltas(Deltas deltas);
    void setApproachVector(approachVector vector);

    void setTrajectory(Trajectory* t);

    QList<machineCoordinates>* getMachineCoordinates();



private:
    /*** Trajectory ***/
    Trajectory *trajectory;

    /*** robot parameters ***/
    double l1, l2, l3, d, e;

    double l4, l5, l6;

    int delta1, delta2, delta5;

    approachVector aV;

    QList<machineCoordinates>* resultCoordinates;



    /*** helper variables ***/
    machineCoordinates mC;
    point3D t;
    point3D p;
    point3D r;

    double S1, C1, S2, C2, S3, C3, S4, C4, S5, C5, S234, C234, S23, C23;
    double CPsi, SPsi, CTheta, STheta;
    double fi23, fi234;
    double a,b,l;

    bool solve();
    void calcPoint_P();
    void calcPoint_R();
    bool checkAngle(double S, double C);
    bool checkSqrt(double value);
    double getFi(double S, double C);

signals:
    void wrongTCP();

};

#endif // KINEMATICS_H
