#include "trajectory.h"

Trajectory::Trajectory(point3D start, point3D end, int pointsNumber)
{
    trajectory = new QList<point3D>();
    for(int i =0; i<pointsNumber; i++)
    {
        point3D pt;
        pt.x = start.x + (start.x - end.x)*i / (pointsNumber-1);
        pt.y = start.y + (start.y - end.y)*i / (pointsNumber-1);
        pt.z = start.z + (start.z - end.z)*i / (pointsNumber-1);
        trajectory->append(pt);
    }
}

Trajectory::~Trajectory()
{
    delete trajectory;
}


QList<point3D>* Trajectory::getTrajectory(){
    return trajectory;
}

void Trajectory::clearTrajectory()
{
    trajectory->clear();
}

point3D Trajectory::getTrajectoryPoint(int index)
{
    return trajectory->at(index);
}

int Trajectory::getTrajectoryLength()
{
    return trajectory->length();
}
