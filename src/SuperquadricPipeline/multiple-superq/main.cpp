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

    // Object Superquadrics
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

    /*******************************************/
    // Read point cloud
    deque<Vector3d> all_points;
    vector<vector<unsigned char>> all_colors;

    ifstream fin(argv[1]);
    if (!fin.is_open())
    {
        cerr << "Unable to open file \"" << argv[1] << "\"";

        return 0;
    }

    Vector3d p(3);
    vector<unsigned int> c_(3);
    vector<unsigned char> c(3);

    string line;
    while (getline(fin,line))
    {
        istringstream iss(line);
        if (!(iss >> p(0) >> p(1) >> p(2)))
            break;
        all_points.push_back(p);

        fill(c_.begin(),c_.end(),120);
        iss >> c_[0] >> c_[1] >> c_[2];
        c[0]=(unsigned char)c_[0];
        c[1]=(unsigned char)c_[1];
        c[2]=(unsigned char)c_[2];

        if (c[0] == c[1] && c[1] == c[2])
         {
             c[0] = 50;
             c[1] = 100;
             c[2] = 0;
         }

        all_colors.push_back(c);
    }

    point_cloud.setPoints(all_points);
    point_cloud.setColors(all_colors);

    /*******************************************/

    // Compute superq
    estim.SetNumericValue("threshold_section2",0.03);
    superqs = estim.computeMultipleSuperq(point_cloud);


    /*******************************************/
    // Compute grasp pose for left hand
    //Vector4d plane;
    //plane << 0.0, 0.0, 1.0, 0.14;
    //grasp_estim.setVector("plane", plane);
    grasp_res = grasp_estim.computeGraspPoses(superqs);

    //Example: Add pose reachable by robot to update Cost         // Uncomment this to include in grasp poses cost reachability of the pose by the robot
    //Vector6d pose_robot;
    //pose_robot << -0.4, 0.0, -0.1, 0.0, 0.0, 0.0;
    //grasp_res.grasp_poses[0].setGraspParamsHat(pose_robot);
    //pose_robot << -0.4, 0.0, -0.1, 0.0, 0.0, 0.0;
    //grasp_res.grasp_poses[1].setGraspParamsHat(pose_robot);
    //grasp_estim.refinePoseCost(grasp_res.grasp_poses);

    // Add poses for grasping
    //vis.addPoses(grasp_res.grasp_poses);
    //vis.addSuperq(hand_superqs);

    /*******************************************/
    // Compute grasp pose for the other hand
    //params_grasp.left_or_right = "left";
    //grasp_res = grasp_estim.computeGraspPoses(params_grasp);

    // Add poses for grasping
    vis.addPoses(grasp_res.grasp_poses);
    //vis.addSuperqHands(grasp_res.hand_superq);           // Uncomment this to visualize the hand ellipsoid

    //PointCloud points_hand;
    //points_hand.setPoints(grasp_res.points_on[0]);  // Uncomment this to visualize points on the hand ellipsoid
    //vis.addPointsHands(points_hand);

    // Outcome visualization
    // Add superquadric to visualizer
    vis.addSuperq(superqs);

    // Add points to visualizer
    // (true/false to show downsample points used for superq estimation)
    vis.addPoints(point_cloud, false);

    // Add plane for grasping
    vis.addPlane(grasp_estim.getPlaneHeight());

    // Visualize
    vis.visualize();

    return 0;
}
