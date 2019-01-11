#ifndef GRASPCOMPUTATION_H
#define GRASPCOMPUTATION_H

#include "superquadricEstimator.h"
#include "graspPoses.h"

typedef Eigen::Matrix<double, 11, 1> Vector11d;
typedef Eigen::Matrix<double, 11, 1> Vector6d;

namespace SuperqGrasp {

/**
* \class LibTemplateCMake::aClass
* \headerfile template-lib.h <TemplateLib/templatelib.h>
*
* \brief A class from LibTemplateCMake namespace.
*
* This class that does a summation.
*/
class graspComputation : public Ipopt::TNLP
{
protected:
  Eigen::Matrix4d H_o2w;
  Eigen::Matrix4d H_h2w;
  Eigen::Matrix4d H_x;
  Eigen::Matrix4d H;
  Eigen::Vector3d euler_hand;
  Eigen::Vector3d euler_obj;
  Eigen::MatrixXd bounds, bounds_constr;

  Vector11d hand, object;
  std::deque<Vector11d> obstacles;
  std::deque<Eigen::Vector3d> points_on;

  int num_superq;
  double aux_objvalue;


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
   double F_v(Vector11d &x, std::deque<Eigen::Vector3d> &points_on);

   /****************************************************************/
    double f_v(Vector11d &x, Eigen::Vector3d &point);

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                         Ipopt::Number *grad_f);


public:


};


class GraspEstimatorApp
{
public:
     SuperqGrasp::GraspPoses computeGraspPoses(SuperqModel::IpoptParam &pars, SuperqModel::Superquadric);
};


}

#endif // LIB_TEMPLATE_CMAKE_H
