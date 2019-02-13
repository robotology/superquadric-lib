/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

 #include <cstdlib>
 #include <string>
 #include <vector>
 #include <cmath>
 #include <algorithm>
 #include <memory>

 #include <yarp/os/all.h>
 #include <yarp/sig/all.h>
 #include <yarp/math/Math.h>
 #include <yarp/dev/Drivers.h>
 #include <yarp/dev/CartesianControl.h>
 #include <yarp/dev/PolyDriver.h>

 #include <superquadricEstimator.h>
 #include <visRenderer.h>
 #include <graspComputation.h>
 #include "src/SuperquadricPipelineDemo_IDL.h"

 using namespace std;
 using namespace SuperqModel;
 using namespace SuperqVis;
 using namespace SuperqGrasp;

 using namespace yarp::os;
 using namespace yarp::sig;
 using namespace yarp::dev;
 using namespace yarp::math;

 /****************************************************************/
enum class WhichHand
{
    HAND_RIGHT,
    HAND_LEFT,
    BOTH
};

/****************************************************************/
string prettyError(const char* func_name, const string &message)
{
    //  Nice formatting for errors
    stringstream error;
    error << "[" << func_name << "] " << message;
    return error.str();
};

 /****************************************************************/
class SuperquadricPipelineDemo : public RFModule, SuperquadricPipelineDemo_IDL
{
    string moduleName;

    RpcClient point_cloud_rpc;
    RpcClient action_render_rpc;
    RpcClient reach_calib_rpc;
    RpcClient table_calib_rpc;

    RpcServer user_rpc;

    bool closing;

    string robot;
    WhichHand grasping_hand;

    PolyDriver left_arm_client, right_arm_client;
    ICartesianControl *icart_right, *icart_left;

    Matrix grasper_specific_transform_right;
    Matrix grasper_specific_transform_left;
    Vector grasper_approach_parameters_right;
    Vector grasper_approach_parameters_left;

    //  visualization parameters
    int x, y, h, w;
    bool fixate_object;

    string best_hand;
    int best_pose_1, best_pose_2;

    // Superquadric-lib objects
    SuperqModel::PointCloud point_cloud;
    vector<Superquadric> superqs;
    GraspResults grasp_res_hand1, grasp_res_hand2;
    SuperqEstimatorApp estim;
    GraspEstimatorApp grasp_estim;
    Visualizer vis;

    Eigen::Vector4d plane;
    Eigen::Vector3d displacement;


   /****************************************************************/
   bool configure(ResourceFinder &rf) override
    {
        moduleName = rf.check("name", Value("superquadric-lib-demo")).toString();
        if(!rf.check("robot"))
        {
            robot = (rf.check("sim")? "icubSim" : "icub");
        }
        else
        {
            robot = rf.find("robot").asString();
        }

        yInfo() << "Opening module for connection with robot" << robot;

        string control_arms = rf.check("control-arms", Value("both")).toString();

        x = rf.check("x", Value(0)).asInt();
        y = rf.check("y", Value(0)).asInt();
        w = rf.check("width", Value(600)).asInt();
        h = rf.check("height", Value(600)).asInt();

        Vector grasp_specific_translation(3, 0.0);
        Vector grasp_specific_orientation(4, 0.0);
        Bottle *list = rf.find("grasp_trsfm_right").asList();
        bool valid_grasp_specific_transform = true;

        if(list)
        {
            if(list->size() == 7)
            {
                for(int i = 0; i < 3; i++) grasp_specific_translation[i] = list->get(i).asDouble();
                for(int i = 0; i < 4; i++) grasp_specific_orientation[i] = list->get(3 + i).asDouble();
            }
            else
            {
                yError() << prettyError(__FUNCTION__, "Invalid grasp_trsfm_right dimension in config. Should be 7.");
                valid_grasp_specific_transform = false;
            }
        }
        else valid_grasp_specific_transform = false;

        if( (!valid_grasp_specific_transform) && ((robot == "icubSim") || (robot == "icub")) )
        {
            yInfo() << "Loading grasp_trsfm_right default value for iCub";
            grasp_specific_translation[0] = -0.01;
            grasp_specific_orientation[1] = 1;
            grasp_specific_orientation[3] = - 38.0 * M_PI/180.0;
        }

        grasper_specific_transform_right = axis2dcm(grasp_specific_orientation);
        grasper_specific_transform_right.setSubcol(grasp_specific_translation, 0,3);
        yInfo() << "Grabber specific transform for right arm loaded\n" << grasper_specific_transform_right.toString();

        grasp_specific_translation.zero();
        grasp_specific_orientation.zero();
        list = rf.find("grasp_trsfm_left").asList();
        valid_grasp_specific_transform = true;

        if(list)
        {
            if(list->size() == 7)
            {
                for(int i = 0; i < 3; i++) grasp_specific_translation[i] = list->get(i).asDouble();
                for(int i = 0; i < 4; i++) grasp_specific_orientation[i] = list->get(3+i).asDouble();
            }
            else
            {
                yError() << prettyError(__FUNCTION__, "Invalid grasp_trsfm_left dimension in config. Should be 7.");
                valid_grasp_specific_transform = false;
            }
        }
        else valid_grasp_specific_transform = false;

        if( (!valid_grasp_specific_transform) && ((robot == "icubSim") || (robot == "icub")) )
        {
            yInfo() << "Loading grasp_trsfm_left default value for iCub";
            grasp_specific_translation[0] = -0.01;
            grasp_specific_orientation[1] = 1;
            grasp_specific_orientation[3] = + 38.0 * M_PI/180.0;
        }

        grasper_specific_transform_left = axis2dcm(grasp_specific_orientation);
        grasper_specific_transform_left.setSubcol(grasp_specific_translation, 0,3);
        yInfo() << "Grabber specific transform for left arm loaded\n" << grasper_specific_transform_left.toString();

        list = rf.find("approach_right").asList();
        grasper_approach_parameters_right.resize(4, 0.0);
        if (list)
        {
            if (list->size() == 4)
            {
                for(int i = 0; i < 4; i++) grasper_approach_parameters_right[i] = list->get(i).asDouble();
            }
            else
            {
                yError() << prettyError(__FUNCTION__, "Invalid approach_right dimension in config. Should be 4.");
            }
        }
        else if((robot == "icubSim") || (robot == "icub"))
        {
            grasper_approach_parameters_right[0] = -0.05;
            grasper_approach_parameters_right[1] = 0.0;
            grasper_approach_parameters_right[2] = -0.05;
            grasper_approach_parameters_right[3] = 0.0;
        }
        yInfo() << "Grabber specific approach for right arm loaded\n" << grasper_approach_parameters_right.toString();

        list = rf.find("approach_left").asList();
        grasper_approach_parameters_left.resize(4, 0.0);
        if (list)
        {
            if (list->size() == 4)
            {
                for(int i=0 ; i<4 ; i++) grasper_approach_parameters_left[i] = list->get(i).asDouble();
            }
            else
            {
                yError() << prettyError(__FUNCTION__, "Invalid approach_left dimension in config. Should be 4.");
            }
        }
        else if((robot == "icubSim") || (robot == "icub"))
        {
            grasper_approach_parameters_left[0] = -0.05;
            grasper_approach_parameters_left[1] = 0.0;
            grasper_approach_parameters_left[2] = +0.05;
            grasper_approach_parameters_left[3] = 0.0;
        }
        yInfo() << "Grabber specific approach for left arm loaded\n" << grasper_approach_parameters_left.toString();

        //  open the necessary ports
        point_cloud_rpc.open("/" + moduleName + "/pointCloud:rpc");
        action_render_rpc.open("/" + moduleName + "/actionRenderer:rpc");
        reach_calib_rpc.open("/" + moduleName + "/reachingCalibration:rpc");
        table_calib_rpc.open("/" + moduleName + "/tableCalib:rpc");
        user_rpc.open("/" + moduleName + "/cmd:rpc");

        //  open clients when using iCub

        if((robot == "icubSim") || (robot == "icub"))
        {
            Property optionLeftArm, optionRightArm;

            optionLeftArm.put("device", "cartesiancontrollerclient");
            optionLeftArm.put("remote", "/" + robot + "/cartesianController/left_arm");
            optionLeftArm.put("local", "/" + moduleName + "/cartesianClient/left_arm");

            optionRightArm.put("device", "cartesiancontrollerclient");
            optionRightArm.put("remote", "/" + robot + "/cartesianController/right_arm");
            optionRightArm.put("local", "/" + moduleName + "/cartesianClient/right_arm");

            if ((control_arms == "both") || (control_arms == "left"))
            {
                if (!left_arm_client.open(optionLeftArm))
                {
                    yError() << prettyError( __FUNCTION__, "Could not open cartesian solver client for left arm");
                    return false;
                }
            }
            if ((control_arms == "both") || (control_arms == "right"))
            {
                if (!right_arm_client.open(optionRightArm))
                {
                    if (left_arm_client.isValid())
                    {
                        left_arm_client.close();
                    }
                    yError() << prettyError( __FUNCTION__, "Could not open cartesian solver client for right arm");
                    return false;
                }
            }
        }

        //  attach callback
        attach(user_rpc);

        fixate_object = false;

        double tol_superq = rf.check("tol_superq", Value(1e-5)).asDouble();
        int print_level_superq = rf.check("print_level_superq", Value(0)).asInt();
        string object_class = rf.check("object_class", Value("default")).toString();
        int optimizer_points = rf.check("optimizer_points", Value(50)).asInt();
        bool random_sampling = rf.check("random_sampling", Value(true)).asBool();

        estim.SetNumericValue("tol", tol_superq);
        estim.SetIntegerValue("print_level", print_level_superq);
        estim.SetStringValue("object_class", object_class);
        estim.SetIntegerValue("optimizer_points", optimizer_points);
        estim.SetBoolValue("random_sampling", random_sampling);

        bool merge_model = rf.check("merge_model", Value(true)).asBool();
        int minimum_points = rf.check("minimum_points", Value(150)).asInt();
        int fraction_pc = rf.check("fraction_pc", Value(8)).asInt();
        double threshold_axis = rf.check("tol_threshold_axissuperq", Value(0.7)).asDouble();
        double threshold_section1 = rf.check("threshold_section1", Value(0.6)).asDouble();
        double threshold_section2 = rf.check("threshold_section2", Value(0.03)).asDouble();

        estim.SetBoolValue("merge_model", merge_model);
        estim.SetIntegerValue("minimum_points", minimum_points);
        estim.SetIntegerValue("fraction_pc", fraction_pc);
        estim.SetNumericValue("threshold_axis", threshold_axis);
        estim.SetNumericValue("threshold_section1", threshold_section1);
        estim.SetNumericValue("threshold_section2", threshold_section2);

        double tol_grasp = rf.check("tol_grasp", Value(1e-5)).asDouble();
        int print_level_grasp = rf.check("print_level_grasp", Value(0)).asInt();
        double constr_tol = rf.check("constr_tol", Value(1e-4)).asDouble();

        grasp_estim.SetNumericValue("tol", tol_grasp);
        grasp_estim.SetIntegerValue("print_level", print_level_grasp);
        grasp_estim.SetNumericValue("constr_tol", constr_tol);
        grasp_estim.SetStringValue("left_or_right", "right");

        list = rf.find("plane_table").asList();
        if (list)
        {
            if (list->size() == 4)
            {
                for(int i=0 ; i<4 ; i++) plane(i) = list->get(i).asDouble();
            }
        }
        else
        {
            plane(0) = 0.0; plane(1) = 0.0; plane(2) = 1.0; plane(3) = 0.15;
        }

        grasp_estim.setVector("plane", plane);
        
        list = rf.find("displacement").asList();
        if (list)
        {
            if (list->size() == 4)
            {
                for(int i=0 ; i<4 ; i++) displacement(i) = list->get(i).asDouble();
            }
        }
        else
        {
            displacement(0) = 0.0; displacement(1) = 0.0; displacement(2) = 1.0;
        }

        grasp_estim.setVector("displacement", displacement);
        //
        // g_params.disp <<  0.0, 0.0, 0.0;
        // g_params.max_superq = 4;
        // g_params.bounds_right << -0.5, 0.0, -0.2, 0.2, -0.3, 0.3, -M_PI, M_PI,-M_PI, M_PI,-M_PI, M_PI;
        // g_params.bounds_left << -0.5, 0.0, -0.2, 0.2, -0.3, 0.3,  -M_PI, M_PI,-M_PI, M_PI,-M_PI, M_PI;
        // g_params.bounds_constr_left.resize(8,2);
        // g_params.bounds_constr_left << -10000, 0.0, -10000, 0.0, -10000, 0.0, 0.01,
        //                                     10.0, 0.0, 1.0, 0.00001, 10.0, 0.00001, 10.0, 0.00001, 10.0;
        // g_params.bounds_constr_right.resize(8,2);
        // g_params.bounds_constr_right << -10000, 0.0, -10000, 0.0, -10000, 0.0, 0.001,
        //                                     10.0, 0.0, 1.0, 0.00001, 10.0, 0.00001, 10.0, 0.00001, 10.0;
        //
        // Superquadric hand;
        // Vector11d hand_vector;
        // hand_vector << 0.03, 0.06, 0.03, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        vis.visualize();

        return true;
    }


    /****************************************************************/
      bool updateModule() override
      {
          return false;
      }

      /****************************************************************/
      double getPeriod() override
      {
          return 1.0;
      }

      /****************************************************************/
      bool interruptModule() override
      {
          point_cloud_rpc.interrupt();
          action_render_rpc.interrupt();
          reach_calib_rpc.interrupt();
          user_rpc.interrupt();
          table_calib_rpc.interrupt();
          closing = true;

          return true;
      }

      /****************************************************************/
      bool close() override
      {
          point_cloud_rpc.close();
          action_render_rpc.close();
          reach_calib_rpc.close();
          user_rpc.close();
          table_calib_rpc.close();

          if (left_arm_client.isValid())
          {
              left_arm_client.close();
          }
          if (right_arm_client.isValid())
          {
              right_arm_client.close();
          }

          return true;
      }

      /****************************************************************/
      bool from_off_file(const string &object_file, const string &hand)
      {
          if (hand == "right")
          {
              grasping_hand = WhichHand::HAND_RIGHT;

              grasp_estim.SetStringValue("left_or_right", hand);
          }
          else if (hand == "left")
          {
              grasping_hand = WhichHand::HAND_LEFT;

              grasp_estim.SetStringValue("left_or_right", hand);
          }
          else if (hand == "both")
          {
              grasping_hand = WhichHand::BOTH;

              grasp_estim.SetStringValue("left_or_right", "right");
          }
          else
          {
              return false;
          }

          deque<Eigen::Vector3d> all_points;
          vector<vector<unsigned char>> all_colors;

          ifstream fin(object_file);
          if (!fin.is_open())
          {
              yError() << "Unable to open file \"" << object_file;

              return 0;
          }

          Eigen::Vector3d p(3);
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
              c[0] = (unsigned char)c_[0];
              c[1] = (unsigned char)c_[1];
              c[2] = (unsigned char)c_[2];

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

          if (point_cloud.getNumberPoints() > 0)
          {
              computeSuperqAndGrasp(true);
              return true;
          }

          return false;
      }

      /****************************************************************/
      bool compute_superq_and_grasp(const string &object_name, const string &hand)
      {
          if (hand == "right")
          {
              grasping_hand = WhichHand::HAND_RIGHT;

              grasp_estim.SetStringValue("left_or_right", hand);
          }
          else if (hand == "left")
          {
              grasping_hand = WhichHand::HAND_LEFT;

              grasp_estim.SetStringValue("left_or_right", hand);
          }
          else if (hand == "both")
          {
              grasping_hand = WhichHand::BOTH;

              grasp_estim.SetStringValue("left_or_right", "right");
          }
          else
          {
              return false;
          }

          bool ok = requestPointCloud(object_name);

          if (ok == true && point_cloud.getNumberPoints() > 0)
          {
              computeSuperqAndGrasp(true);
          }

          return ok;
      }

      /****************************************************************/
      bool grasp()
      {
          Vector best_pose;

          if (grasping_hand == WhichHand::BOTH)
          {
              if (best_hand == "right")
              {
                  Eigen::VectorXd pose;
                  pose.resize(7);
                  pose.head(3) = grasp_res_hand1.grasp_poses[grasp_res_hand1.best_pose].getGraspPosition();
                  pose.tail(4) = grasp_res_hand1.grasp_poses[grasp_res_hand1.best_pose].getGraspAxisAngle();
                  best_pose = eigenToYarp(pose);
              }
              else if (best_hand == "left")
              {
                  Eigen::VectorXd pose;
                  pose.resize(7);
                  pose.head(3) = grasp_res_hand1.grasp_poses[grasp_res_hand2.best_pose].getGraspPosition();
                  pose.tail(4) = grasp_res_hand1.grasp_poses[grasp_res_hand2.best_pose].getGraspAxisAngle();
                  best_pose = eigenToYarp(pose);
              }
          }
          else
          {
              Eigen::VectorXd pose;
              pose.resize(7);
              pose.head(3) = grasp_res_hand1.grasp_poses[grasp_res_hand1.best_pose].getGraspPosition();
              pose.tail(4) = grasp_res_hand1.grasp_poses[grasp_res_hand1.best_pose].getGraspAxisAngle();
              best_pose = eigenToYarp(pose);
          }

          executeGrasp(best_pose, best_hand);
      }

      /************************************************************************/
      bool requestPointCloud(const string &object)
      {
         Bottle cmd_request;
         Bottle cmd_reply;

         //  if fixate_object is given, look at the object before acquiring the point cloud
         if (fixate_object)
         {
             if(action_render_rpc.getOutputCount() < 1)
             {
                 yError() << prettyError( __FUNCTION__,  "requestRefreshPointCloud: no connection to action rendering module");
                 return false;
             }

             cmd_request.addVocab(Vocab::encode("look"));
             cmd_request.addString(object);
             cmd_request.addString("wait");

             action_render_rpc.write(cmd_request, cmd_reply);
             if (cmd_reply.get(0).asVocab() != Vocab::encode("ack"))
             {
                 yError() << prettyError( __FUNCTION__,  "Didn't manage to look at the object");
                 return false;
             }
         }

         point_cloud.deletePoints();
         cmd_request.clear();
         cmd_reply.clear();

         yarp::sig::PointCloud<DataXYZRGBA> pc;

         cmd_request.addString("get_point_cloud");
         cmd_request.addString(object);

         if(point_cloud_rpc.getOutputCount() < 1)
         {
             yError() << prettyError( __FUNCTION__,  "requestRefreshPointCloud: no connection to point cloud module");
             return false;
         }

         point_cloud_rpc.write(cmd_request, cmd_reply);

         //  cheap workaround to get the point cloud
         Bottle* pcBt = cmd_reply.get(0).asList();
         bool success = pc.fromBottle(*pcBt);

         deque<Eigen::Vector3d> acquired_points;

         for (size_t i = 0; i < pc.size(); i++)
         {
            Eigen::Vector3d point;
            point(0)=pc(0,i).x; point(1)=pc(1,i).y; point(2)=pc(2,i).z;
            acquired_points.push_back(point);
         }

         if (success && (pc.size() > 0))
         {
            point_cloud.setPoints(acquired_points);
            return true;
         }
         else
            return false;
      }

      /************************************************************************/
      void computeSuperqAndGrasp(bool choose_hand)
      {
          // Reset visualizer for new computations
          vis.resetSuperq();
          vis.resetPoses();
          vis.resetPoints();

          // Visualize acquired point cloud
          vis.addPoints(point_cloud, false);

          // Compute superq
          superqs = estim.computeSuperq(point_cloud);

          // Visualize downsampled point cloud and estimated superq
          vis.addPoints(point_cloud, true);
          vis.addSuperq(superqs);

          getTable();

          // Compute grasp pose
          grasp_res_hand1 = grasp_estim.computeGraspPoses(superqs);

          // Show computed grasp pose and plane
          vis.addPoses(grasp_res_hand1.grasp_poses);
          vis.addPlane(grasp_estim.getPlaneHeight());

          // Compute and show grasp pose for the other hand
          if (grasping_hand == WhichHand::BOTH)
          {
              grasp_estim.SetStringValue("left_or_right", "left");
              grasp_res_hand2 = grasp_estim.computeGraspPoses(superqs);
              vis.addPoses(grasp_res_hand1.grasp_poses, grasp_res_hand2.grasp_poses);
          }

          if (choose_hand)
          {
              // TODO extend to R1 (see cardinal-grasp-pointss)
              if (grasping_hand == WhichHand::BOTH)
              {
                  if (left_arm_client.isValid())
                  {
                      left_arm_client.view(icart_left);

                      computePoseHat(grasp_res_hand2, icart_left);
                  }

                  if (right_arm_client.isValid())
                  {
                      right_arm_client.view(icart_right);

                      computePoseHat(grasp_res_hand1, icart_right);
                  }
              }
              else if (grasping_hand == WhichHand::HAND_RIGHT)
              {
                  if (right_arm_client.isValid())
                  {
                      right_arm_client.view(icart_right);

                      computePoseHat(grasp_res_hand1, icart_right);
                  }
              }
              else if (grasping_hand == WhichHand::HAND_LEFT)
              {
                  if (left_arm_client.isValid())
                  {
                      left_arm_client.view(icart_left);

                      computePoseHat(grasp_res_hand1, icart_left);
                  }
              }

              grasp_estim.refinePoseCost(grasp_res_hand1);

              if (grasping_hand == WhichHand::BOTH)
              {
                  grasp_estim.refinePoseCost(grasp_res_hand2);
                  vis.addPoses(grasp_res_hand1.grasp_poses, grasp_res_hand2.grasp_poses);
              }
              else
                  vis.addPoses(grasp_res_hand1.grasp_poses);

              if (grasping_hand == WhichHand::BOTH)
              {
                  int best_right = grasp_res_hand1.best_pose;
                  int best_left = grasp_res_hand2.best_pose;

                  if (grasp_res_hand1.grasp_poses[best_right].cost < grasp_res_hand2.grasp_poses[best_left].cost)
                  {
                      best_hand = "right";
                      vis.highlightBestPose("right", "both", best_right);
                  }
                  else
                  {
                      best_hand = "left";
                      vis.highlightBestPose("left", "both", best_left);
                  }
              }
              else if (grasping_hand == WhichHand::HAND_RIGHT)
              {
                  int best_right = grasp_res_hand1.best_pose;
                  vis.highlightBestPose("right", "both", best_right);
              }
              else if (grasping_hand == WhichHand::HAND_LEFT)
              {
                  int best_left = grasp_res_hand1.best_pose;
                  vis.highlightBestPose("left", "both", best_left);
              }
          }
      }

      /****************************************************************/
      void getTable()
      {
          bool table_ok = false;
          if (robot != "icubSim" && table_calib_rpc.getOutputCount() > 0)
          {
              Bottle table_cmd, table_rply;
              table_cmd.addVocab(Vocab::encode("get"));
              table_cmd.addString("table");

              table_calib_rpc.write(table_cmd, table_rply);
              if (Bottle *payload = table_rply.get(0).asList())
              {
                  if (payload->size() >= 2)
                  {
                      plane(0) = 0.0;
                      plane(1) = 0.0;
                      plane(2) = 1.0;

                      plane(3) = payload->get(1).asDouble();
                      table_ok = true;

                      grasp_estim.setVector("plane", plane);
                  }
              }
          }
          if (!table_ok)
          {
              yWarning() << " Unable to retrieve table height, using default.";
          }

          yInfo() << " Using table height =" << - plane[3];
      }

      /****************************************************************/
      void setGraspContext(ICartesianControl *icart)
      {
          //  set up the context for the grasping planning and execution
          //  enable all joints
          Vector dof;
          icart->getDOF(dof);
          Vector new_dof(10, 1);
          new_dof(1) = 0.0;
          icart->setDOF(new_dof, dof);
          icart->setPosePriority("position");
          icart->setInTargetTol(0.001);
        }

      /****************************************************************/
      Vector eigenToYarp(Eigen::VectorXd &v)
      {
          Vector x;
          x.resize(v.size());

          for (size_t i = 0; i< x.size(); i++)
          {
              x[i] = v[i];
          }

          return x;
      }

      /****************************************************************/
      Eigen::VectorXd yarpToEigen(Vector &v)
      {
          Eigen::VectorXd x;
          x.resize(v.size());

          for (size_t i = 0; i< x.size(); i++)
          {
              x[i] = v[i];
          }

          return x;
      }

      /****************************************************************/
      void computePoseHat(GraspResults &grasp_res, ICartesianControl *icart)
      {
          for (size_t i = 0; i < grasp_res.grasp_poses.size(); i++)
          {
              int context_backup;
              icart->storeContext(&context_backup);

              //  set up the context for the computation of the candidates
              setGraspContext(icart);

              Eigen::VectorXd desired_pose = grasp_res.grasp_poses[i].getGraspPosition();
              Eigen::VectorXd desired_or = grasp_res.grasp_poses[i].getGraspAxisAngle();

              Vector x_d = eigenToYarp(desired_pose);
              Vector o_d = eigenToYarp(desired_or);

              Vector x_d_hat, o_d_hat, q_d_hat;

              //yDebug() << "X desired "<<x_d.toString();
              //yDebug() << "O desired "<<o_d.toString();

              bool success = icart->askForPose(x_d, o_d, x_d_hat, o_d_hat, q_d_hat);

              Eigen::VectorXd pose_hat = yarpToEigen(x_d_hat);
              Eigen::VectorXd or_hat = yarpToEigen(o_d_hat);

              //yDebug() << "X hat "<<x_d_hat.toString();
              //yDebug() << "O hat "<<o_d_hat.toString();

              Eigen::VectorXd robot_pose;
              robot_pose.resize(7);
              robot_pose.head(3) = pose_hat;
              robot_pose.tail(4) = or_hat;

              for (size_t i = 0; i < grasp_res.grasp_poses.size(); i++)
              {
                  grasp_res.grasp_poses[i].setGraspParamsHat(robot_pose);
              }

              //  restore previous context
              icart->restoreContext(context_backup);
              icart->deleteContext(context_backup);
          }
      }

      /****************************************************************/
      bool executeGrasp(Vector &pose, string &best_hand)
      {
          if(robot == "icubSim" )
          {
              //  simulation context, suppose there is no actionsRenderingEngine running
              if (best_hand == "right")
              {
                  int context_backup;
                  icart_right->storeContext(&context_backup);
                  setGraspContext(icart_right);
                  Vector previous_x(3), previous_o(4);
                  icart_right->getPose(previous_x, previous_o);
                  icart_right->goToPoseSync(pose.subVector(0, 2), pose.subVector(3,6));
                  icart_right->waitMotionDone();
                  icart_right->goToPoseSync(previous_x, previous_o);
                  icart_right->waitMotionDone();
                  icart_right->restoreContext(context_backup);
                  icart_right->deleteContext(context_backup);
                  return true;
              }
              else if (best_hand == "left")
              {
                int context_backup;
                icart_left->storeContext(&context_backup);
                setGraspContext(icart_left);
                Vector previous_x(3), previous_o(4);
                icart_left->getPose(previous_x, previous_o);
                icart_left->goToPoseSync(pose.subVector(0, 2), pose.subVector(3,6));
                icart_left->waitMotionDone();
                icart_left->goToPoseSync(previous_x, previous_o);
                icart_left->waitMotionDone();
                icart_left->restoreContext(context_backup);
                icart_left->deleteContext(context_backup);
                return true;
              }
          }
          else
          {
              //  communication with actionRenderingEngine/cmd:io
              //  grasp("cartesian" x y z gx gy gz theta) ("approach" (-0.05 0 +-0.05 0.0)) "left"/"right"
              Bottle command, reply;

              command.addString("grasp");
              Bottle &ptr = command.addList();
              ptr.addString("cartesian");
              ptr.addDouble(pose(0));
              ptr.addDouble(pose(1));
              ptr.addDouble(pose(2));
              ptr.addDouble(pose(3));
              ptr.addDouble(pose(4));
              ptr.addDouble(pose(5));
              ptr.addDouble(pose(6));


              Bottle &ptr1 = command.addList();
              ptr1.addString("approach");
              Bottle &ptr2 = ptr1.addList();
              if (grasping_hand == WhichHand::HAND_LEFT)
              {
                  for(int i=0 ; i<4 ; i++) ptr2.addDouble(grasper_approach_parameters_left[i]);
                  command.addString("left");
              }
              else
              {
                  for(int i=0 ; i<4 ; i++) ptr2.addDouble(grasper_approach_parameters_right[i]);
                  command.addString("right");
              }

              yInfo() << command.toString();
              action_render_rpc.write(command, reply);
              if (reply.toString() == "[ack]")
              {
                  return true;
              }
              else
              {
                  return false;
              }
          }

      }

      /************************************************************************/
      bool attach(RpcServer &source)
      {
        return this->yarp().attachAsServer(source);
      }
};

int main(int argc, char *argv[])
{
    Network yarp;
    ResourceFinder rf;
    rf.setDefaultContext("superquadric-lib");
    rf.setDefaultConfigFile("config-icub-superq-lib.ini");
    rf.configure(argc, argv);

    if (!yarp.checkNetwork())
    {
        yError() << prettyError(__FUNCTION__, "YARP network not detected. Check nameserver");
        return EXIT_FAILURE;
    }

    SuperquadricPipelineDemo disp;

    return disp.runModule(rf);
}
