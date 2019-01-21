#ifndef SUPERQGRASP_H
#define SUPERQGRASP_H

#include <Eigen/Dense>

#include "superquadric.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;

namespace SuperqGrasp {

/**
* \class SuperqModel::Superquadric
* \headerfile superquadric.h <SuperquadricModel/include/superquadric.h>
*
* \brief A class from SuperqModel namespace.
*
* This class implements a generic Superquadric with 11 parameters.
*/
class GraspPoses
{
public:
    Eigen::VectorXd position;
    Eigen::Vector4d axisangle;
    Eigen::Vector3d ea;
    Eigen::Matrix3d axes;
    Eigen::VectorXd params;

    double cost;
    std::string hand;

    /**
    * Constructor
    */
    GraspPoses();

    /**
    * Destructory
    */
    ~GraspPoses() { }

    /**
     * Set all pose parameters
     * @param p is a eigen vector of dimension 6 or 7
     * containing the grasping pose parameters
     * @return true/false if the parameters are consistent
     */
    bool setGraspParams(const Eigen::VectorXd &p);

    /**
     * Set grasp position coordinates
     * @param c is a eigen vector of dimension 3
     * containing the grasp position
     * @return true/false if the parameters are consistent
     */
    bool setGraspPosition(Eigen::Vector3d &c);

    /**
     * Set grasp orientation
     * @param o is a eigen vector of dimension 4
     * containing the grasp orientation as axis angle
     * @return true/false if the parameters are consistent
     */
    bool setGraspOrientation(Eigen::Vector4d &o);

    /**
     * Set grasp orientation
     * @param o is a eigen vector of dimension 3
     * containing the grasp orientation as euler angles
     * @return true/false if the parameters are consistent
     */
    bool setGraspOrientation(Eigen::Vector3d &a);

    /**
     * Get all superquadric parameters
     * @return a 11D vector containing the parameters
     */
    Eigen::VectorXd getGraspParams();

    /**
     * Get grasping position
     * @return a 3D vector containing the  enter
     */
    Eigen::Vector3d getGraspPosition();

    /**
     * Get grasp pose orientation as axis angle
     * return a 4D vector containing the axis angle
     */
    Eigen::Vector4d getGraspAxisAngle();

    /**
     * Get grasp pose orientation as euler angles
     * @return a 3D vector containing the Euler angles
     */
    Eigen::Vector3d getGraspEulerZYZ();

    /**
     * Get grasp pose axes
     * @return a 3x3 Matrix containing the axes
     */
    Eigen::Matrix3d getGraspAxes();

    /*********************************************/
    std::string getHandName();

    /*********************************************/
    void setHandName(std::string h);


};

}

#endif
