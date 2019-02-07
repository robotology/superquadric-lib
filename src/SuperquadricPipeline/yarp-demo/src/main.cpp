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

 Mutex mutex;

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
    WhichHand grasping_hand_best;

    PolyDriver left_arm_client, right_arm_client;
    ICartesianControl *icart;

    // TODO Check if to put it in Eigen
    Vector planar_obstacle; // plane to avoid, typically a table (format (a b c d) following plane equation a.x+b.y+c.z+d=0)
    Vector grasper_bounding_box; // bounding box of the grasper (x_min x_max y_min _y_max z_min z_max) expressed in the robot grasper frame used by the controller
    double obstacle_safety_distance; // minimal distance to respect between the grasper and the obstacle
    double palm_width;
    double finger_length;
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
        if(list)
        {
            if(list->size() == 4)
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
        if(list)
        {
            if(list->size() == 4)
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

        list = rf.find("grasp_bounding_box").asList();
        if(list)
        {
            if(list->size() == 6)
            {
                for(int i = 0; i < 6; i++) grasper_bounding_box[i] = list->get(i).asDouble();
            }
            else
            {
                yError() << prettyError(__FUNCTION__, "Invalid grasp_bounding_box dimension in config. Should be 6.");
            }
        }
        yInfo() << "Grabber bounding box loaded\n" << grasper_bounding_box.toString();

        list = rf.find("planar_obstacle").asList();
        if(list)
        {
            if(list->size() == 4)
            {
                for(int i = 0; i < 4; i++) planar_obstacle[i] = list->get(i).asDouble();
            }
            else
            {
                planar_obstacle[0] = 0.0;
                planar_obstacle[1] = 0.0;
                planar_obstacle[2] = 1;
                planar_obstacle[3] = -(-0.15);
                yError() << prettyError(__FUNCTION__, "Invalid planar_obstacle dimension in config. Should be 4.");
            }
        }
        yInfo() << "Planar obstacle loaded\n" << planar_obstacle.toString();


        obstacle_safety_distance = rf.check("obstacle_safety_distance", Value(0.0)).asDouble();
        yInfo() << "Obstacle safety distance loaded=" << obstacle_safety_distance;

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
          //user_rpc.interrupt();
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
          //user_rpc.close();
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

          /*******************************************/
          // Read point cloud
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
              computeSuperqAndGrasp(false);
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
            point(0)=pc(0,i).x; point(1)=pc(01,i).y; point(2)=pc(2,i).z;
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
          vis.clean();
          vis.addPoints(point_cloud, false);
          // Compute superq
          superqs = estim.computeSuperq(point_cloud);

          vis.addPoints(point_cloud, true);
          vis.addSuperq(superqs);

          grasp_res_hand1 = grasp_estim.computeGraspPoses(superqs);
          vis.addPoses(grasp_res_hand1.grasp_poses);
          vis.addPlane(grasp_estim.getPlaneHeight());

          if (grasping_hand == WhichHand::BOTH)
          {
              grasp_estim.SetStringValue("left_or_right", "left");
              grasp_res_hand2 = grasp_estim.computeGraspPoses(superqs);
          }

          vis.addPoses(grasp_res_hand2.grasp_poses);
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
