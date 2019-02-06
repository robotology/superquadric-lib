/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
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

 }
