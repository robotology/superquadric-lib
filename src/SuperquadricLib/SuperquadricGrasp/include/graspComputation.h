/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#ifndef GRASPCOMPUTATION_H
#define GRASPCOMPUTATION_H

#include "superquadricEstimator.h"
#include "graspPoses.h"

typedef Eigen::Matrix<double, 6, 2> Matrix62d;

namespace SuperqGrasp {

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

class graspComputation : public Ipopt::TNLP
{
protected:
    /* Homogeneous trasformation representing object pose
    in world reference frame  (o2w) */
    Eigen::Matrix4d H_o2w;
    /* Homogeneous trasformation representing hand pose
    in world reference frame  (h2w) */
    Eigen::Matrix4d H_h2w;

    /* Displacement between the hand reference frame and the hand ellipsoid */
    Eigen::Vector3d displacement;
    /* Plane parameters */
    Eigen::Vector4d plane;

    /* Bounds for the grasping pose */
    Matrix62d bounds;
    /* Bounds for constraints of the optimization problem */
    Eigen::MatrixXd bounds_constr;

    /* Auxiliary vector */
    Eigen::VectorXd g;

    /* vector containing the hand ellipsoid in final pose */
    Vector6d solution_vector;

    /* Cone axis for pose constraints */
    Eigen::Vector3d d_x, d_y, d_z;
    /* Cone apertures for constraints on orientation */
    double theta_x, theta_y, theta_z;

    /* Vector containing parameters of hand and object superquadrics */
    Vector11d hand, object;
    /* Deque of vector containing parameters of obstacles superquadrics */
    std::deque<Vector11d> obstacles;

    /* Number of obstacles */
    int num_superq;
    /* Maximum number of obstacles */
    int max_superq;
    /* Number of points sampled on hand */
    int n_hands;
    /* Auxiliary value */
    double aux_objvalue;
    /* For which hand compute the grasping pose */
    std::string l_o_r;
    /* Used just for aligning pose (currently off)*/
    double top_grasp;

    /* Struct GrasPoses contains what's relevant as solution:
    * - grasping pos
    * - hand ellipsoid in the final pose
    * - points sampled on the hand ellipsoid in the final pose*/
    GraspPoses solution;

    /* Vector containing robot pose */
    Vector6d robot_pose;

public:

    /* Points sampled on the hand ellipsoid */
    std::deque<Eigen::Vector3d> points_on;

    /* Final values of distance between obstacles */
    double final_F_value;
    std::deque<double> final_obstacles_value;

    /****************************************************************/
    void init(GraspParams &g_params);

    /****************************************************************/
    Eigen::Vector3d computePointsHand(Vector11d &hand, const int &j, const int &l, const std::string &str_hand, const double &theta);

    /****************************************************************/
    double sign(const double &v);

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                              Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style);

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u);

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                 bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                                 Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda);

   /****************************************************************/
   bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                                Ipopt::Number &obj_value);

   /****************************************************************/
   double F(const Ipopt::Number *x, std::deque<Eigen::Vector3d> &points_on, bool new_x);

   /****************************************************************/
   double f(const Ipopt::Number *x, Eigen::Vector3d &point);

   /****************************************************************/
   double F_v(const Vector6d &x);

   /****************************************************************/
    double f_v(const Vector6d &x, const Eigen::Vector3d &point);

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                         Ipopt::Number *grad_f);

    /****************************************************************/
    double coneImplicitFunction(const Eigen::Vector3d &point, const Eigen::Vector3d &d, double theta);

    /****************************************************************/
    double G_v(const Vector6d &x, const int &i, Ipopt::Index m);

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g);

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values);

    /****************************************************************/
    void configure(GraspParams &g_params);

    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                          const Ipopt::Number *x, const Ipopt::Number *z_L,
                          const Ipopt::Number *z_U, Ipopt::Index m,
                          const Ipopt::Number *g, const Ipopt::Number *lambda,
                          Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                          Ipopt::IpoptCalculatedQuantities *ip_cq);

    /****************************************************************/
    GraspPoses get_result() const;

    /****************************************************************/
    SuperqModel::Superquadric get_hand() const;

    /****************************************************************/
    double get_final_F() const;

    /****************************************************************/
    std::deque<double> get_final_constr_values() const;

    /****************************************************************/
    double computeObstacleValues(const Ipopt::Number *x, const int &k);

    /****************************************************************/
    double computeObstacleValues_v(const Vector6d &pose_hand, const int &k);

    /****************************************************************/
    std::deque<double> computeFinalObstacleValues(const Vector6d &pose_hand);

    /****************************************************************/
    bool notAlignedPose(const Eigen::Matrix4d &final_H);

    /****************************************************************/
    void alignPose(Eigen::Matrix4d &final_H);

    /*****************************************************************/
    double f_v2(const Vector11d &obj, const Eigen::Vector3d &point_tr);
};

struct GraspResults
{
    /* Superquadric representing the hand ellipsoid in the final pose */
    std::vector<SuperqModel::Superquadric> hand_superq;
    /* Grasping pose */
    std::vector<GraspPoses> grasp_poses;
    /*Points sampled on hand ellipsoid in the final pose */
    std::vector<std::deque<Eigen::Vector3d>> points_on;
    /* Final cost function */
    std::vector<double> F_final;
    /* Final average distance w.r.t to the obstacles */
    std::vector<std::deque<double>> F_final_obstacles;

};
class GraspEstimatorApp
{
    SuperqModel::IpoptParam pars;
    SuperqGrasp::GraspParams g_params;
public:
     GraspEstimatorApp();
     /*****************************************************************/
     GraspResults computeGraspPoses(std::vector<SuperqModel::Superquadric> &superqs);
     /*****************************************************************/
     void refinePoseCost(std::vector<SuperqGrasp::GraspPoses> &pose_computed);
     /*****************************************************************/
     double getPlaneHeight();
};

}

#endif
