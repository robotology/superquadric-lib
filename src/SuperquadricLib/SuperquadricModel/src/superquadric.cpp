#include "superquadric.h"

#include <iostream>

using namespace std;
using namespace Eigen;
using namespace SuperqModel;

Superquadric::Superquadric(int num_params=11)
{
    params.resize(num_params);
    params.setZero();
    n_params=num_params;
}

bool Superquadric::setSuperqParams(VectorXd &p)
{
    bool params_ok=true;

    params_ok=params_ok && (p.size()==n_params);

    if (params_ok)
        params_ok=params_ok && (p(0) > 0 && p(1) > 0 && p(2) > 0);

    if (params_ok)
        params=p;

    return params_ok;

}

VectorXd Superquadric::getSuperqParams()
{
    return params;
}

bool Superquadric::setSuperqDims(Vector3d &d)
{
    bool params_ok=true;


    params_ok=params_ok && (d(0) > 0 && d(1) > 0 && d(2) > 0);

    if (params_ok)
        params.head(3)=d;

    return params_ok;

}

Vector3d Superquadric::getSuperqDims()
{
    return params.head(3);
}

bool Superquadric::setSuperqExps(Vector2d &e)
{
    bool params_ok=true;


    params_ok=params_ok && ((e(0) > 0 && e(1) > 0) &&  (e(0) < 2 && e(1) < 2));

    if (params_ok)
        params.segment(3,2)=e;

    return params_ok;

}

Vector2d Superquadric::getSuperqExps()
{
    return params.segment(3,2);
}


bool Superquadric::setSuperqCenter(Vector3d &c)
{
    params.segment(5,3)=c;

    return true;

}

Vector3d Superquadric::getSuperqCenter()
{
    return params.segment(5,3);
}


bool Superquadric::setSuperqOrientation(Vector3d &o)
{
    params.segment(8,3)=o;

    axes = AngleAxisd(o(0), Vector3d::UnitZ())*
           AngleAxisd(o(1), Vector3d::UnitY())*
           AngleAxisd(o(2), Vector3d::UnitZ());

    return true;

}

Vector3d Superquadric::getSuperqEulerZYZ()
{
    return params.segment(8,3);
}

bool Superquadric::setSuperqOrientation(Vector4d &o)
{
    bool params_ok=true;
    params_ok=params_ok && (o.head(3).norm()==1);

    if (params_ok)
    {
       axes = AngleAxisd(o(3), o.head(3));

       Vector3d ea = axes.eulerAngles(2,1,2);

       params.segment(8,3)=ea;

    }

    return params_ok;

}

Vector4d Superquadric::getSuperqAxisAngle()
{
    AngleAxisd aa_superq(axes);

    Vector4d aa;

    aa.head(3)=aa_superq.axis();
    aa(3)=aa_superq.angle();

    return aa;
}







