#ifndef POINTCLOUD_H
#define POINTCLOUD_H

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
    /**
    * Constructor
    */
    PointCloud();

    /**
    * Destructory
    */
    virtual ~PointCloud();


};


}

#endif
