/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#ifndef OPTIONS_H
#define OPTIONS_H

#include "superquadric.h"

typedef Eigen::Matrix<double, 6, 2> Matrix62d;

struct IpoptParam
{
    double tol;
    double constr_tol;
    int acceptable_iter;
    std::string mu_strategy;
    int max_iter;
    double max_cpu_time;
    std::string nlp_scaling_method;
    std::string hessian_approximation;
    int print_level;
    std::string object_class;
    int optimizer_points;
    bool random_sampling;
};

struct MultipleParams
{
    bool merge_model;
    int minimum_points;
    int fraction_pc;
    double threshold_section1;
    double threshold_section2;
    double threshold_axis;
    double debug;
};

struct GraspParams
{
    /* For which hand compute the grasping pose */
    std::string left_or_right;
    /* Bounds for the grasping pose of the right hand */
    Matrix62d bounds_right;
    /* Bounds for the grasping pose of the left hand */
    Matrix62d bounds_left;
    /* Bounds for constraints of the optimization problem
     of the right hand */
    Eigen::MatrixXd bounds_constr_right;
    /* Bounds for constraints of the optimization problem
     of the left hand */
    Eigen::MatrixXd bounds_constr_left;
    /* Maximum number of obstacles superquadric to be consider */
    int max_superq;
    /* Plane parameters */
    Eigen::Vector4d pl;
    /* Displacement between the hand reference frame and the hand ellipsoid */
    Eigen::Vector3d disp;
    /* Superquadric representing the object to be grasped */
    SuperqModel::Superquadric object_superq;
    /* Superquadric representing the object to be grasped */
    std::vector<SuperqModel::Superquadric> obstacle_superqs;
    /* Superquadric representing the robot hand */
    SuperqModel::Superquadric hand_superq;
    /* Superquadrics representing obstacles */
    std::vector<SuperqModel::Superquadric> object_superqs;

};

class Options
{

protected:

  IpoptParam pars;
  GraspParams g_params;
  MultipleParams m_pars;

public:

  Options();

  /****************************************************************/
  void SetNumericValue(const std::string &tag, const double &value);

  /****************************************************************/
  void SetIntegerValue(const std::string &tag, const int &value);

  /****************************************************************/
  void SetStringValue(const std::string &tag, const std::string &value);
};





#endif
