/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#include <SuperquadricLibGrasp/graspPoses.h>

using namespace std;
using namespace Eigen;
using namespace SuperqModel;
using namespace SuperqGrasp;

/*********************************************/
GraspPoses::GraspPoses()
{
    cost = 1e8;
    hand = "right";
}

/*********************************************/
bool GraspPoses::setGraspParams(const VectorXd &p)
{
    bool params_ok = true;

    params_ok = params_ok && ((p.size() == 6) || (p.size() == 7));

    if (params_ok)
        params = p;
    else
        return false;

    position = params.head(3);

    if (p.size() == 6)
    {
        ea = params.segment(3,3);
        axes = AngleAxisd(ea(0), Vector3d::UnitZ())*
               AngleAxisd(ea(1), Vector3d::UnitY())*
               AngleAxisd(ea(2), Vector3d::UnitZ());
        AngleAxisd aa_pose(axes);
        axisangle.head(3) = aa_pose.axis();
        axisangle(3) = aa_pose.angle();
    }
    else if (p.size() == 7)
    {
        axisangle = params.segment(3,4);
        axes = AngleAxisd(axisangle(3), axisangle.head(3));
        ea = axes.eulerAngles(2,1,2);
    }

    return params_ok;
}

/*********************************************/
VectorXd GraspPoses::getGraspParams()
{
    return params;
}

/*********************************************/
bool GraspPoses::setGraspParamsHat(const VectorXd &p)
{
    bool params_ok = true;

    params_ok = params_ok && ((p.size() == 6) || (p.size() == 7));

    if (params_ok)
        params_hat = p;
    else
        return false;

    return params_ok;
}

/*********************************************/
VectorXd GraspPoses::getGraspParamsHat()
{
    return params_hat;
}

/*********************************************/
bool GraspPoses::setGraspPosition(Vector3d &d)
{
    params.head(3) = d;
    position = params.head(3);

    return true;
}

/*********************************************/
Vector3d GraspPoses::getGraspPosition()
{
    return position;
}

/*********************************************/
bool GraspPoses::setGraspOrientation(Vector3d &o)
{
    params.segment(3,3) = o;
    axes = AngleAxisd(o(0), Vector3d::UnitZ())*
           AngleAxisd(o(1), Vector3d::UnitY())*
           AngleAxisd(o(2), Vector3d::UnitZ());
    ea = o;

    return true;

}

/*********************************************/
Vector3d GraspPoses::getGraspEulerZYZ()
{
    return ea;
}

/*********************************************/
bool GraspPoses::setGraspOrientation(Vector4d &o)
{
    bool params_ok = true;
    params_ok = params_ok && (o.head(3).norm() == 1);

    if (params_ok)
    {
       axes = AngleAxisd(o(3), o.head(3));
       ea = axes.eulerAngles(2,1,2);
       params.segment(3,3) = ea;
       axisangle = o;
    }

    return params_ok;
}

/*********************************************/
Vector4d GraspPoses::getGraspAxisAngle()
{
    return axisangle;
}

/*********************************************/
Matrix3d GraspPoses::getGraspAxes()
{
    return axes;
}

/*********************************************/
string GraspPoses::getHandName()
{
    return hand;
}

/*********************************************/
void GraspPoses::setHandName(string h)
{
    hand = h;
}
