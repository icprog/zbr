#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "trajectory.h"
#include "qmath.h"

struct approachVector
{
    double theta;
    double psi;
};
struct machineCoordinates
{
    double fi1;
    double fi2;
    double fi3;
    double fi4;
    double fi5;
};

class Kinematics
{
public:
    Kinematics();
    ~Kinematics();






private:
    /*** robot parameters ***/
    double l1, l2, l3, d, e;

    double l4, l5, l6;

    int delta1, delta2, delta5;

    approachVector aV;
    machineCoordinates mC;

    /*** TCP coordinates ***/
    point3D t;

    /*** helper variables ***/

    point3D p;
    point3D r;

   double S1, C1, S2, C2, S3, C3, S4, C4, S5, C5, S234, C234, S23, C23;
   double CPsi, SPsi, CTheta, STheta;
   double fi23, fi234;
   double a,b;

    void solve();
    void calcPoint_P();
    void calcPoint_R();
    double getFi(double S, double C);



};

#endif // KINEMATICS_H
