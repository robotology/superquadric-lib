#ifndef GRASPCOMPUTATION_H
#define GRASPCOMPUTATION_H

#include "superquadricEstimator.h"
#include "graspPoses.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 2> Matrix62d;

namespace SuperqGrasp {

struct GraspParams
{
    std::string left_or_right;
    Matrix62d bounds_right;
    Matrix62d bounds_left;
    Eigen::MatrixXd bounds_constr_right;
    Eigen::MatrixXd bounds_constr_left;
    int max_superq;
    Eigen::Vector4d pl;
    Eigen::Vector3d disp;
};

class graspComputation : public Ipopt::TNLP
{
protected:
    Eigen::Matrix4d H_o2w;
    Eigen::Matrix4d H_h2w;
    Eigen::Matrix4d H_x;
    Eigen::Matrix4d H;
    Eigen::Vector3d d_x, d_y, d_z;
    Eigen::Vector3d displacement;
    Eigen::Vector4d plane;
    Matrix62d bounds;
    Eigen::MatrixXd bounds_constr;
    Eigen::VectorXd g;
    Vector6d x_tmp;
    Eigen::Vector4d point_tr, point_tmp;

    double theta_x, theta_y, theta_z;

    Vector11d hand, object;
    std::deque<Vector11d> obstacles;
    std::deque<Eigen::Vector3d> points_on;

    int num_superq;
    int max_superq;
    double aux_objvalue;
    std::string l_o_r;
    double top_grasp;

    Vector6d solution;
    Vector6d robot_pose;
    double final_F_value;
    std::deque<double> final_obstacles_value;

public:
    /****************************************************************/
    void init(SuperqModel::Superquadric &object_superq, SuperqModel::Superquadric &hand_superq,
                                const std::deque<SuperqModel::Superquadric> &obstacles_superq, int &n_handpoints, const std::string &str_hand);


    /****************************************************************/
    Eigen::Vector3d computePointsHand(Vector11d &hand, int j, int l, const std::string &str_hand, double &theta);

    /****************************************************************/
    double sign(double &v);

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
   double F_v(Vector6d &x);

   /****************************************************************/
    double f_v(Vector6d &x, Eigen::Vector3d &point);

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                         Ipopt::Number *grad_f);

    /****************************************************************/
    double coneImplicitFunction(const Eigen::Vector3d &point, const Eigen::Vector3d &d, double theta);

    /****************************************************************/
    double G_v(Vector6d &x, int i, Ipopt::Index m);

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
    Vector6d get_result() const;

    /****************************************************************/
    Vector11d get_hand() const;

    /****************************************************************/
    double get_final_F() const;

    /****************************************************************/
    std::deque<double> get_final_constr_values() const;

    /****************************************************************/
    double computeObstacleValues(const Ipopt::Number *x, int k);

    /****************************************************************/
    double computeObstacleValues_v(Vector6d &pose_hand, int k);

    /****************************************************************/
    std::deque<double> computeFinalObstacleValues(Vector6d &pose_hand);

    /****************************************************************/
    bool notAlignedPose(Eigen::Matrix4d &final_H);

    /****************************************************************/
    void alignPose(Eigen::Matrix4d &final_H);

    /*****************************************************************/
    double f_v2(Vector11d &obj, Eigen::Vector3d &point_tr);
};

class GraspEstimatorApp
{
public:
     SuperqGrasp::GraspPoses computeGraspPoses(SuperqModel::IpoptParam &pars, SuperqModel::Superquadric);
};

}

#endif // LIB_TEMPLATE_CMAKE_H
