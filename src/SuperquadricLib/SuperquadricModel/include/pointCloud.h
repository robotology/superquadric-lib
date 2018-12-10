#ifndef POINTCLOUD_H
#define POINTCLOUD_H

namespace PointCloud {

/**
* \class PointCloud::3Dpoints
* \headerfile pointcloud.h <SuperquadricModel/include/pointcloud.h>
*
* \brief A class from PointCloud namespace.
*
* This class contains a 3D point cloud used for estimating the superquadric.
*/
class 3Dpoints
{
public:
    /**
    * Constructor
    */
    3Dpoints();

    /**
    * Destructory
    */
    virtual ~3Dpoints();


};


}

#endif
