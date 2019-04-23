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

#ifndef SUPERQMODEL_H
#define SUPERQMODEL_H

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 11, 1> Vector11d;

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

    /* Number of superquadric parameters (11 in this implementation)*/
    int n_params;
    /* Vector with all the parameters */
    Vector11d params;
    /* Superquadric dimensions */
    Eigen::Vector3d dim;
    /* Superquadric exponents */
    Eigen::Vector2d exp;
    /* Superquadric orienation expressed as axisangle */
    Eigen::Vector4d axisangle;
    /* Superquadric orienation expressed as euler angles */
    Eigen::Vector3d ea;
    /* Superquadric center */
    Eigen::Vector3d center;
    /* Superquadric orienation expressed as rotation matrix */
    Eigen::Matrix3d axes;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
    * Constructor
    */
    Superquadric();

    /**
    * Destructory
    */
    ~Superquadric() { }

    /**
     * Set all superquadric parameters
     * @param p is a eigen vector of dimension num_params
     * containing the superquadric parameters
     * @return true/false if the parameters are consistent
     */
    bool setSuperqParams(const Vector11d &p);

    /**
     * Set  superquadric dimensions
     * @param d is a eigen vector of dimension 3
     * containing the superquadric dimensions
     * @return true/false if the parameters are consistent
     */
    bool setSuperqDims(const Eigen::Vector3d &d);

    /**
     * Set  superquadric exponents
     * @param e is a eigen vector of dimension 2
     * containing the superquadric exponents
     * @return true/false if the parameters are consistent
     */
    bool setSuperqExps(const Eigen::Vector2d &e);

    /**
     * Set  superquadric center coordinates
     * @param c is a eigen vector of dimension 3
     * containing the superquadric center
     * @return true/false if the parameters are consistent
     */
    bool setSuperqCenter(const Eigen::Vector3d &c);


    /**
     * Set  superquadric orientation
     * @param o is a eigen vector of dimension 3 or 4
     * containing the superquadric orientation as euler angles zyz
     * if dimensions is 3, or containing the axis angle SetOrientationMarker
     * if dimensions is 4.
     * @return true/false if the parameters are consistent
     */
    bool setSuperqOrientation(const Eigen::VectorXd &a);

    /**
     * Get all superquadric parameters
     * @return a 11D vector containing the parameters
     */
    Vector11d getSuperqParams();

    /**
     * Get  superquadric dimensions
     * @return a 3D vector containing the dimensions
     */
    Eigen::Vector3d getSuperqDims();

    /**
     * Get  superquadric exponents
     * @return a 2D vector containing the exponents
     */
    Eigen::Vector2d getSuperqExps();

    /**
     * Get  superquadric center coordinates
     * @return a 3D vector containing the  enter
     */
    Eigen::Vector3d getSuperqCenter();

    /**
     * Get  superquadric orientation
     * return a 4D vector containing the axis angle
     */
    Eigen::Vector4d getSuperqAxisAngle();

    /**
     * Get  superquadric orientation
     * @return a 3D vector containing the Euler angles
     */
    Eigen::Vector3d getSuperqEulerZYZ();

    /**
     * Get superquadric axes
     * @return a 3x3 Matrix containing the axes
     */
    Eigen::Matrix3d getSuperqAxes();

    /**
     * Compute the inside-outside function of the superquadric
     * @param pose is the pose of the superquadric
     * @param point is a point where we want to evaluate the inside-ouside function
     * @return the value of the inside-outside function
     */
    double insideOutsideF(const Vector11d &pose, const Eigen::Vector3d &point);
};

}

#endif
