#ifndef TYPEDEFS_H
#define TYPEDEFS_H



struct point3D
{
    double x;
    double y;
    double z;
};

struct approachVector
{
    double theta;
    double psi;
};
struct machineCartesianCoordinates
{
    point3D p1;
    point3D p1pr;
    point3D p2;
    point3D p2pr;
    point3D p3;
    point3D p4;
    point3D p5;
};

struct machineCoordinates
{
    double fi1;
    double fi2;
    double fi3;
    double fi4;
    double fi5;
    machineCartesianCoordinates cartesian;
};



struct robotParamsRegional
{
    double l1;
    double l2;
    double l3;
    double d;
    double e;
};

struct robotParamsLocal
{
    double l4;
    double l5;
    double l6;
};

struct Deltas
{
    int d1;
    int d2;
    int d5;
};

#endif // TYPEDEFS_H
