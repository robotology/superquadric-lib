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

 #ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/SVD>
#include <deque>
#include <vector>

typedef Eigen::Matrix<double, 3, 2>  Matrix32d;

namespace SuperqModel {

/**
* \class SuperqModel::3Dpoints
* \headerfile pointcloud.h <SuperquadricModel/include/pointcloud.h>
*
* \brief A class from SuperqModel namespace.
*
* This class contains a 3D point cloud used for estimating the superquadric.
*/
class PointCloud
{
public:

    int n_points;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points_for_vis;
    std::vector<std::vector<unsigned char>> colors;
    Eigen::Vector3d barycenter;
    Eigen::Matrix3d orientation;
    Matrix32d bounding_box;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
    * Constructor
    */
    PointCloud();

    /**
    * Destructory
    */
    ~PointCloud();

    /**
     * set points of the point cloud
     * @param p is a deque of 3d or 6d eigen vectors
     * @return true is number of points > 0 and if they are 3d or 6d
     */
    bool setPoints(const std::deque<Eigen::Vector3d> &p);

    /**
     * set colors of the point cloud
     * @param c is a vector of vector of char
     * @return true the number of colors is equal to the number of points
     */
    /*********************************************/
    bool setColors(const std::vector<std::vector<unsigned char>> &c);

    /**
     * get the number of points of the point cloud
     * @return the number of points
     */
    int getNumberPoints();

    /**
     * delete the points of the point cloud
     */
    void deletePoints();

    /**
     * get the bounding box of the point cloud
     * @return a 3d matrix containing the bounding box of the point cloud
     */
    /*********************************************/
    Eigen::MatrixXd getBoundingBox();

    /**
     * get the bounding box of the point cloud in the superquadric reference frame
     * @param the orientation of the point cloud
     * @return a 3d matrix containing the bounding box of the point cloud
     */
    /*********************************************/
    Eigen::MatrixXd getBoundingBoxSuperqFrame(const Eigen::Matrix3d &orientation);

    /**
     * get the barycenter of the point cloud
     * @return a 3d vector containing the barycenter of the point cloud
     */
    Eigen::Vector3d getBarycenter();

    /**
     * get the axes of the point cloud
     * @return a 3x3 matrix containing the axes of the point cloud
     */
    Eigen::Matrix3d getAxes();


    /**
     * Subsample the point cloud
     * @param desired_points_num is the desired number of points after the downsampling
     */
    void subSample(const int &desired_points_num, const bool &random);

    bool readFromFile(const char* file_name);

    bool readFromFile(const std::string &file_name);

};


}

#endif
