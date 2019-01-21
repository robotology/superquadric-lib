#include "superquadricEstimator.h"
#include "visRenderer.h"
#include "graspComputation.h"

#include <cstdlib>
#include <cmath>
#include <iostream>


using namespace std;
using namespace Eigen;
using namespace SuperqModel;
using namespace SuperqVis;
using namespace SuperqGrasp;


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
    SuperqEstimatorApp estim;

    // Superq Estimator
    GraspEstimatorApp grasp_estim;

    // Grasp pose
    GraspPoses grasp_pose;

    // Params for solver in superq estimator
    IpoptParam iparams_superq;
    iparams_superq.tol=1e-5;
    iparams_superq.acceptable_iter=0;
    iparams_superq.mu_strategy="adaptive";
    iparams_superq.max_iter=1000000;
    iparams_superq.max_cpu_time=5.0;
    iparams_superq.nlp_scaling_method="gradient-based";
    iparams_superq.hessian_approximation="limited-memory";
    iparams_superq.print_level=0;
    iparams_superq.object_class="default";
    iparams_superq.optimizer_points=50;
    iparams_superq.random_sampling=true;

    // Params for solver in grasp estimator
    IpoptParam iparams_grasp;
    iparams_grasp.tol=1e-5;
    iparams_grasp.constr_tol=1e-4;
    iparams_grasp.acceptable_iter=0;
    iparams_grasp.mu_strategy="adaptive";
    iparams_grasp.max_iter=1000000;
    iparams_grasp.max_cpu_time=5.0;
    iparams_grasp.nlp_scaling_method="gradient-based";
    iparams_grasp.hessian_approximation="limited-memory";
    iparams_grasp.print_level=0;


    /*******************************************/
    // Read point cloud
    deque<Vector3d> all_points;
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

        if (c[0]==c[1] && c[1]==c[2])
         {
             c[0]=50;
             c[1]=100;
             c[2]=0;
         }

        all_colors.push_back(c);
    }

    point_cloud.setPoints(all_points);
    point_cloud.setColors(all_colors);

    /*******************************************/
    // Compute superq
    superq=estim.computeSuperq(iparams_superq, point_cloud);

    // Params for grasp computation
    GraspParams params_grasp;
    params_grasp.left_or_right="left";
    params_grasp.pl << 0.0, 0.0, 1.0, 0.18;
    params_grasp.disp <<  0.05, 0.0, 0.0;
    params_grasp.n_hands = 48;
    params_grasp.object_superq = superq;
    params_grasp.max_superq = 4;
    params_grasp.bounds_left << -0.5, 0.0, -0.2, 0.2, -0.3, 0.2, -M_PI, M_PI,-M_PI, M_PI,-M_PI, M_PI;
    params_grasp.bounds_right << -0.5, 0.0, -0.2, 0.2, -0.2, 0.3,  -M_PI, M_PI,-M_PI, M_PI,-M_PI, M_PI;
    params_grasp.bounds_constr_left.resize(8,2);
    params_grasp.bounds_constr_left << -10000, 0.0, -10000, 0.0, -10000, 0.0, 0.01,
                                        10.0, 0.0, 1.0, 0.001, 10.0, 0.001, 10.0, 0.001, 10.0;
    params_grasp.bounds_constr_right.resize(8,2);
    params_grasp.bounds_constr_right << -10000, 0.0, -10000, 0.0, -10000, 0.0, 0.01,
                                        10.0, 0.0, 1.0, 0.001, 10.0, 0.001, 10.0, 0.001, 10.0;

    Superquadric hand;
    Vector11d hand_vector;
    hand_vector << 0.03, 0.06, 0.03, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    hand.setSuperqParams(hand_vector);
    params_grasp.hand_superq = hand;

    /*******************************************/
    // Compute grasp pose
    grasp_pose=grasp_estim.computeGraspPoses(iparams_grasp, params_grasp);

    /*******************************************/
    // Outcome visualization

    // VTK need to receive a vector of Superquadrics
    vector<Superquadric> superqs;
    //superq.setSuperqParams(params);
    superqs.push_back(superq);

    // Add superquadric to visualizer
    vis.addSuperq(superqs);

    // Add points to visualizer
    // (true/false to show downsample points used for superq estimation)
    vis.addPoints(point_cloud, true);

    // Visualize
    vis.visualize();

    return 0;
}
