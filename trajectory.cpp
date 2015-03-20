#include "trajectory.h"

Trajectory::Trajectory()
{
    trajectory = new QList<point3D>();
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
bool Trajectory::isEmpty()
{
    return trajectory->isEmpty();
}

void Trajectory::addPoint(point3D p, int pointsNumber)
{
    if(trajectory->isEmpty())
        return;
    point3D start = trajectory->last();
    for(int i =0; i<pointsNumber; i++)
    {
        point3D pt;
        pt.x = start.x + (p.x - start.x)*i / (pointsNumber-1);
        pt.y = start.y + (p.y - start.y)*i / (pointsNumber-1);
        pt.z = start.z + (p.z - start.z)*i / (pointsNumber-1);
        trajectory->append(pt);
    }
}

void Trajectory::addBegin(point3D begin)
{
    trajectory->append(begin);
}

int Trajectory::getTrajectoryLength()
{
    return trajectory->length();
}
