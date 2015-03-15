#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <QList>
#include "typedefs.h"

class Trajectory
{
public:
    Trajectory(point3D start, point3D end, int pointsNumber);
    ~Trajectory();
    QList<point3D>* getTrajectory();
    point3D getTrajectoryPoint(int index);
    int getTrajectoryLength();


private:
    QList<point3D> *trajectory;

};

#endif // TRAJECTORY_H
