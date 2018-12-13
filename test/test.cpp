#include "superquadric.h"

#include <cstdlib>
#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace SuperqModel;

int main()
{
    int num_params=11;
    VectorXd params(num_params);
    params(0)=0.1;
    params(1)=0.2;
    params(2)=0.3;
    Superquadric superq(num_params);

    superq.setSuperqParams(params);

    if( (superq.getSuperqParams() - params).norm() > 0.0)
    {
        cerr << "[ERROR] superquadric parameters not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    Vector3d center;
    center<<-0.1, -0.2, -0.3;

    superq.setSuperqCenter(center);

    if( (superq.getSuperqCenter() - center).norm() > 0.0)
    {
        cerr << "[ERROR] superquadric center not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    Vector3d dimensions;
    dimensions<<10, 20, 30;

    superq.setSuperqDims(dimensions);

    if( (superq.getSuperqDims() - dimensions).norm() > 0.0)
    {
        cerr << "[ERROR] superquadric dimensions not set correctly"<<endl;
        return EXIT_FAILURE;
    }


    Vector2d exp;
    exp<< 1.5, 1.5;

    superq.setSuperqExps(exp);

    if( (superq.getSuperqExps() - exp).norm() > 0.0)
    {
        cerr << "[ERROR] superquadric exps not set correctly"<<endl;
        return EXIT_FAILURE;
    }


    Vector3d ea;
    ea<<0.7, 0.0, 0.0;

    superq.setSuperqOrientation(ea);

    if( (superq.getSuperqEulerZYZ() - ea).norm() > 0.0)
    {
        cerr << "[ERROR] superquadric Euler angles not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    Vector4d aa;
    aa<<1.0, 0.0, 0.0, 0.7;

    superq.setSuperqOrientation(aa);

    if( (superq.getSuperqAxisAngle() - aa).norm() > 1e-10)
    {
        cerr << "[ERROR] superquadric axis angles not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
