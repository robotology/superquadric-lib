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
    params(0)=1.0;
    params(1)=2.0;
    params(2)=3.0;

    VectorXd s_params(num_params);

    Superquadric superq(num_params);

    s_params=superq.getSuperqParams();

    cout<< "old params "<<endl;
    cout<<s_params;

    superq.setSuperqParams(params);

    s_params=superq.getSuperqParams();

    cout<< "new params "<<endl;
    cout<<s_params;

    Vector3d center;
    center<<0.1, 0.2, 0.3;

    superq.setSuperqCenter(center);

    cout<< "center "<<endl;
    cout<<superq.getSuperqCenter;

    Vector3d dimensions;
    dimensions<<10, 20, 30;


    Vector2d exp;
    exp<< 1.5, 1.5;

    Vector3d ea;
    ea<<0.7, 0.7, 0.7;

    superq.setSuperqCenter(center);

}
