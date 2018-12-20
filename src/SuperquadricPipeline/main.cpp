#include "superquadric.h"
#include "visRenderer.h"

#include <cstdlib>
#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace SuperqModel;
using namespace SuperqVis;

int main()
{
    // Create params of superquadric
    int num_params=11;
    VectorXd params(num_params);
    params(0)=0.1;
    params(1)=0.2;
    params(2)=0.3;
    params(3)=0.1;
    params(4)=1.0;

    VectorXd s_params(num_params);

    // Create Superquadric instance
    Superquadric superq(num_params);

    // VTK need to receive a vector of Superquadrics
    vector<Superquadric> superqs;
    superq.setSuperqParams(params);
    superqs.push_back(superq);

    //VTK visualizer
    Visualizer vis;

    // Add superquadric to visualizer
    vis.addSuperq(superqs);

    // Visualize
    vis.visualize();


    return 0;
}
