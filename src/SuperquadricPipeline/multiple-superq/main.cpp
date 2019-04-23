/******************************************************************************
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation; either version 2 of the License, or (at your option) any later
* version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.                                                                     *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#include <SuperquadricLibModel/superquadricEstimator.h>
#include <SuperquadricLibVis/visRenderer.h>
#include <SuperquadricLibGrasp/graspComputation.h>

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
    // Check on parameters
    if (argc < 2)
    {
        cout<<endl;
        cout << "     Missing path file: " << endl;
        cout<< "     Provide the complete path of the file containing a partial point cloud " << endl<<endl;
        return 0;
    }

    /*******************************************/
    // PointCloud class
    PointCloud point_cloud;

    // Object Superquadrics
    vector<Superquadric> superqs;

    // Superq Estimator
    SuperqEstimatorApp estim;

    // Grasping pose Estimator
    GraspEstimatorApp grasp_estim;

    // Grasp pose
    GraspResults grasp_res;
    vector<GraspPoses> grasp_poses;
    vector<Superquadric> hand_superqs;

    // VTK visualizer
    Visualizer vis;

    /*******************************************/
    // Read point cloud
    if (!point_cloud.readFromFile(argv[1]))
        return 0;

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
    //grasp_estim.refinePoseCost(grasp_res);

    // Add poses for grasping
    //vis.addPoses(grasp_res.grasp_poses);

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
