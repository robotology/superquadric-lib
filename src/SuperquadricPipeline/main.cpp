#define HAVE_CSTDDEF
#include <IpTNLP.hpp>
#undef HAVE_CSTDDEF

#include "superquadricEstimator.h"
#include "visRenderer.h"

#include <cstdlib>
#include <cmath>
#include <iostream>



using namespace std;
using namespace Eigen;
using namespace SuperqModel;
using namespace SuperqVis;


int main(int argc, char* argv[])
{
    /*******************************************/
    //Instance everything

    // PointCloud class
    PointCloud point_cloud;

    // Create Superquadric
    Superquadric superq;

    // VTK visualizer
    Visualizer vis;

    // Superq Estimator
    EstimatorApp estim;

    // Params for solver in estimator
    IpoptParam params;
    params.tol=1e-5;
    params.acceptable_iter=0;
    params.mu_strategy="adaptive";
    params.max_iter=10000;
    params.max_cpu_time=5.0;
    params.nlp_scaling_method="gradient-based";
    params.hessian_approximation="limited-memory";
    params.print_level=0;
    params.object_class="default";
    params.optimizer_points=50;

    /*******************************************/
    // Read point cloud
    deque<VectorXd> all_points;
    vector<vector<unsigned char>> all_colors;

    ifstream fin(argv[1]);
    if (!fin.is_open())
    {
        cerr<<"Unable to open file \""<<argv[1]<<"\"";
        return 0;
    }

    Vector3d p(3);
    vector<unsigned int> c_(3);
    vector<unsigned char> c(3);

    string line;
    while (getline(fin,line))
    {
        istringstream iss(line);
        if (!(iss>>p(0)>>p(1)>>p(2)))
            break;
        all_points.push_back(p);

        fill(c_.begin(),c_.end(),120);
        iss>>c_[0]>>c_[1]>>c_[2];
        c[0]=(unsigned char)c_[0];
        c[1]=(unsigned char)c_[1];
        c[2]=(unsigned char)c_[2];
        all_colors.push_back(c);
    }

    point_cloud.setPoints(all_points);

    /*******************************************/
    // Compute superq
    superq=estim.computeSuperq(params, point_cloud);

    /*******************************************/
    // Outcome visualization

    // VTK need to receive a vector of Superquadrics
    vector<Superquadric> superqs;
    //superq.setSuperqParams(params);
    superqs.push_back(superq);

    // Add superquadric to visualizer
    vis.addSuperq(superqs);

    // Add points to visualizer // Fix this
    //vis.addPoints(point_cloud);

    // Visualize
    vis.visualize();

    return 0;
}
