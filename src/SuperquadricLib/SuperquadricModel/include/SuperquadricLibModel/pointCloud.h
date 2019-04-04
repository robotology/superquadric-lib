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