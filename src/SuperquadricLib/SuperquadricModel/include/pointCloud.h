#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <deque>

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
    std::deque<Eigen::VectorXd> points;
    Eigen::Vector3d barycenter;
    Eigen::Matrix3d orientation;
    Eigen::MatrixXd bounding_box;

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
    bool setPoints(std::deque<Eigen::VectorXd> &p);

    /**
     * get the number of points of the point cloud
     * @return the number of points
     */
    int getNumberPoints();

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
    void subSample(int desired_points_num);

    /**
     * Remove outliers using dbscan algorithm
     * @param radius and minpts are two parameters of dbscan for tuning the outlier removal
     */
    //void removeOutliers(double &radius=0.1, int &minpts=10);

};


}

#endif
