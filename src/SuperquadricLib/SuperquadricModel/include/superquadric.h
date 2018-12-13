#ifndef SUPERQMODEL_H
#define SUPERQMODEL_H

#include <Eigen/Dense>

namespace SuperqModel {

/**
* \class SuperqModel::Superquadric
* \headerfile superquadric.h <SuperquadricModel/include/superquadric.h>
*
* \brief A class from SuperqModel namespace.
*
* This class implements a generic Superquadric with 11 parameters.
*/
class Superquadric
{
public:

    int n_params;
    Eigen::VectorXd params;
    Eigen::Vector3d dim;
    Eigen::Vector2d exp;
    Eigen::Vector4d axisangle;
    Eigen::Vector3d eulerZYZ;
    Eigen::Vector3d center;
    Eigen::Matrix3d axes;

    /**
    * Constructor
    */
    Superquadric(int num_params);

    /**
    * Destructory
    */
    ~Superquadric() { }

    /**
     * Set all superquadric parameters
     * return true/false if the parameters are consistent
     */
    bool setSuperqParams(Eigen::VectorXd &p);

    /**
     * Set  superquadric dimensions
     * return true/false if the parameters are consistent
     */
    bool setSuperqDims(Eigen::Vector3d &d);

    /**
     * Set  superquadric exponents
     * return true/false if the parameters are consistent
     */
    bool setSuperqExps(Eigen::Vector2d &e);

    /**
     * Set  superquadric center coordinates
     * return true/false if the parameters are consistent
     */
    bool setSuperqCenter(Eigen::Vector3d &c);

    /**
     * Set  superquadric orientation
     * return true/false if the parameters are consistent
     */
    bool setSuperqOrientation(Eigen::Vector4d &o);

    /**
     * Set  superquadric orientation
     * return true/false if the parameters are consistent
     */
    bool setSuperqOrientation(Eigen::Vector3d &a);

    /**
     * Get all superquadric parameters
     * return a 11D vector containing the parameters
     */
    Eigen::VectorXd  getSuperqParams();

    /**
     * Get  superquadric dimensions
     * return a 3D vector containing the dimensions
     */
    Eigen::Vector3d getSuperqDims();

    /**
     * Get  superquadric exponents
     * return a 2D vector containing the exponents
     */
    Eigen::Vector2d getSuperqExps();

    /**
     * Get  superquadric center coordinates
     * return a 3D vector containing the  enter
     */
    Eigen::Vector3d getSuperqCenter();

    /**
     * Get  superquadric orientation
     * return a 4D vector containing the axis angle
     */
    Eigen::Vector4d getSuperqAxisAngle();

    /**
     * Get  superquadric orientation
     * return a 3D vector containing the Euler angles
     */
    Eigen::Vector3d getSuperqEulerZYZ();

    /**
     * Get superquadric axes
     * return a 3x3 Matrix containing the axes
     */
    Eigen::Matrix3d getSuperqAxes();
};

}

#endif
