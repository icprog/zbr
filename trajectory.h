#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <QList>
#include "typedefs.h"

class Trajectory
{
public:
    Trajectory();
    ~Trajectory();
    QList<point3D>* getTrajectory();
    void clearTrajectory();
    void addPoint(point3D p, int pointsNumber);
    void addBegin(point3D begin);
    bool isEmpty();
    point3D getTrajectoryPoint(int index);
    int getTrajectoryLength();


private:
    QList<point3D> *trajectory;

};

#endif // TRAJECTORY_H
