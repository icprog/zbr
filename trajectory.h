#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <QList>
struct point3D
{
    double x;
    double y;
    double z;
};

class Trajectory
{
public:
    Trajectory::Trajectory(point3D start, point3D end, int pointsNumber);
    ~Trajectory();
    QList<point3D>* getTrajectory();

private:
    QList<point3D> *trajectory;

};

#endif // TRAJECTORY_H
