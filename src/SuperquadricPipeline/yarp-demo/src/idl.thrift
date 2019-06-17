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

 service SuperquadricPipelineDemo_IDL
 {
     /**
     * Compute superquadric and grasping pose from offline
     * @param object_file is the object off file with point cloud
     * @param hand is the desired hand for computation, can be right, left or both
     *@return true/false on success/failure.
     */
     bool from_off_file(1: string object_file, 2: string hand);

     bool compute_superq_and_pose(1: string object_name, 2: string hand);

     bool grasp();

     bool drop();

     bool set_single_superq(1: string value);

     bool take_tool();

     bool open_hand();

 }
