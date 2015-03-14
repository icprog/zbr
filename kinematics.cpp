#include "kinematics.h"

Kinematics::Kinematics()
{

}

Kinematics::~Kinematics()
{

}

void Kinematics::solve()
{
    double l = l5+l6;
    CPsi = cos(aV.psi);
    SPsi = sin(aV.psi);
    CTheta = cos(aV.theta);
    STheta = cos(aV.theta);

    calcPoint_P();

    S1 = (1 / (p.x*p.x + p.y*p.y))*(e*p.x + delta1*p.y*sqrt(p.x*p.x+p.y*p.y-e*e));
    C1 = (1 / (p.x*p.x + p.y*p.y))*(-1*e*p.y + delta1*p.x*sqrt(p.x*p.x+p.y*p.y-e*e));
    mC.fi1 = getFi(S1,C1);

    S5 = CTheta * (SPsi * C1 - CPsi * S1);
    C5 = delta5*sqrt(1-S5*S5);
    mC.fi5 = getFi(S5,C5);

    S234 = STheta / C5;
    C234 = (CTheta/C5)*(CPsi*C1 + SPsi * S1);
    fi234 = getFi(S234,C234);

    calcPoint_R();

    a = -1*l1 +delta1*sqrt(r.x*r.x + r.y*r.y - e*e);
    b= (a*a +r.z*r.z +l2*l2 -l3*l3)/(2*l2);
    S2 = (r.z * b + delta2*a*sqrt(a*a+r.z*r.z-b*b))/(a*a + r.z*r.z);
    C2 = (a * b + delta2*r.z*sqrt(a*a+r.z*r.z-b*b))/(a*a + r.z*r.z);
    mC.fi2 = getFi(S2,C2);

    S3 = (r.z * C2 - a * S2)/l3;
    C3 = (a * C3 + r.z * S2 - l2)/l3;
    mC.fi3 = getFi(S3,C3);

    S23 = (r.z - l2*S2)/l3;
    C23 = (a - l2*C2)/l3;
    f23 = getFi(S23,C23);

    S4 = S234*C23 - C234*S23;
    C4 = C234*C23 + S234*S23;
    mC.fi4 = getFi(S4,C4);






}

void Kinematics::calcPoint_P()
{
    p.x = t.x - l * CTheta*CPsi;
    p.y = t.y - l * CTheta*SPsi;
    p.z = t.z - l * STheta;
}

void Kinematics::calcPoint_R()
{
    r.x = p.x - l4*C1*C234;
    r.y = p.y - l4*S1*C234;
    r.z = p.z - l4*S234;
}

double Kinematics::getFi(double S, double C)
{
    if(abs(S)<=abs(C))
        return asin(S);
    else return acos(C);
}
