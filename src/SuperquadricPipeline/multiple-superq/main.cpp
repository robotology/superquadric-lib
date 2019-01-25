/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

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
    // PointCloud class
    PointCloud point_cloud;

    //PointCloud points_hand; // Uncomment this to visualize points on hand ellipsoid in the final pose

    // Object Superquadric
    vector<Superquadric> superqs;

    // VTK visualizer
    Visualizer vis;

    // Superq Estimator
    SuperqEstimatorApp estim;

    // Grasping pose Estimator
    GraspEstimatorApp grasp_estim;

    // Grasp pose
    GraspResults grasp_res;
    vector<GraspPoses> grasp_poses;
    vector<Superquadric> hand_superqs;

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

    MultipleParams m_pars;
    m_pars.merge_model=true;
    m_pars.minimum_points=150;
    m_pars.fraction_pc=8;
    m_pars.threshold_axis=0.7;
    m_pars.threshold_section1=0.6;
    m_pars.threshold_section2=0.03;
    m_pars.debug=false;

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
    superqs=estim.computeMultipleSuperq(iparams_superq, m_pars, point_cloud);

    // Outcome visualization
    // Add superquadric to visualizer
    vis.addSuperq(superqs);

    // Add points to visualizer
    // (true/false to show downsample points used for superq estimation)
    vis.addPoints(point_cloud, true);

    // Add poses for grasping
    vis.addPoses(grasp_poses);
    //vis.addSuperq(hand_superqs);

    // points_hand.setPoints(grasp_res.points_on);  // Uncomment this to visualize points on the hand ellipsoid
    // vis.addPoints(points_hand, false);

    // Visualize
    vis.visualize();

    return 0;
}
