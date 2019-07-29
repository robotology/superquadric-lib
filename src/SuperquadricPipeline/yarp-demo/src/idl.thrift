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
 * @file idl.thirft
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

 /**
 * Bottle
 *
 * IDL structure to set/show advanced parameters.
 */
 struct Bottle
 {
 } (
    yarp.name = "yarp::os::Bottle"
    yarp.includefile="yarp/os/Bottle.h"
 )

struct PointD {
  1: double x;
  2: double y;
  3: double z;
}

 service SuperquadricPipelineDemo_IDL
 {

     /**
     * Compute superquadric and grasping pose from offline
     * @param object_file is the object off file with point cloud
     * @param hand is the desired hand for computation, can be right, left or both
     *@return true/false on success/failure.
     */
     bool from_off_file(1: string object_file, 2: string hand);

     /**
     * Compute superquadric and grasping pose of a labelled object
     * @param object_name is the object label
     * @param hand is the desired hand for computation, can be right, left or both
     *@return true/false on success/failure.
     */
     bool compute_superq_and_pose(1: string object_name, 2: string hand);

     bool compute_superq_and_pose_from_position(1: list <double> &position, 2: string &hand)

     bool save_pcloud(1: string base_dir, 2: string obj_name)

     bool grasp();

     bool drop();

     bool home();

     bool stopMotion();

     bool quit();

     string get_superq_mode();

     bool set_single_superq(1: bool value);

     /**
     * Get the image region where sfm is computed
     *@return a list with [u_i, v_i, u_f, v_f]
     */
     list<i32> get_sfm_region();

     /**
     * Set the image region where sfm is computed
     * @param u_i, v_i are the (u,v) coordinates of the initial pixel of the region
     * @param u_f, v_f are the (u,v) coordinates of the final pixel of the region
     *@return true/false on success/failure.
     */
     bool set_sfm_region(1: double u_i, 2: double v_i, 3: double u_f, 4: double v_f);

     bool take_tool(1: bool go_home = false);

     bool open_hand(1: string hand);

     list<PointD> get_tool_trajectory();

     bool set_tool_trajectory(1: list<PointD> point);

     bool set_approach(1: string hand, 2: list<double> value)

     list<double> get_approach(1: string hand)

     map<string,PointD> get_best_grasp_position()

     bool refine_best_grasp_position(1: PointD position)

     /* Get/set point cloud filtering params */

     map<string,double> get_pc_filter_params()
     bool set_pc_filter_params(1: string param_name, 2: double value)


     /* Get/set super quadric model params */

     map<string,double> get_sq_model_params()
     bool set_sq_model_param(1: string param_name, 2: double value)

     /* Get/set super quadric grasp params */

     map<string,double> get_sq_grasp_params()
     bool set_sq_grasp_param(1: string param_name, 2: double value)

     list<double> get_hand_sq_params()
     bool set_hand_sq_params(1: list<double> values)

 }
