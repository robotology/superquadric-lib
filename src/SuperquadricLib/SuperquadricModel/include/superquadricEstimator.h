#ifndef SUPERQESTIMATOR_H
#define SUPERQESTIMATOR_H

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <IpReturnCodes.hpp>

#include "superquadric.h"
#include "pointCloud.h"

namespace SuperqModel {

/**
* \class SuperqModel::Superq_NLP
* \headerfile superqEstimator.h <SuperquadricModel/include/superqEstimator.h>
*
* \brief A class from SuperqModel namespace.
*
* This class implements an IpOpt optimizer for reconstructing a superquadric from point clouds.
*/
class SuperqEstimator : public Ipopt::TNLP
{
protected:
  Eigen::VectorXd x0;
  Eigen::MatrixXd bounds;
  std::string obj_class;
  double aux_objvalue;

  /** Get info for the nonlinear problem to be solved with ipopt
  * @param n is the dimension of the variable
  * @param m is the number of constraints
  * @param nnz_jac_g is the dimensions of the jacobian
  * @param nnz_h_lag is an ipopt variable
  * @param index_styl is an ipopt variable
  * @return true
  */
  /****************************************************************/
  bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                    Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style);

  /** Compute bounds variable from the point cloud for speeding up optimization */
  /****************************************************************/
  void computeBounds();

  /** Get variable bounds for the nonlinear problem to be solved with ipopt
  * @param n is the dimension of the variable
  * @param m is the number of constraints
  * @param x_l is the lower bound of the variable
  * @param x_u is the upper bound of the variable
  * @param g_l is the lower bound of the constraints
  * @param g_u is the upper bound of the constraints
  * @return true
  */
  /****************************************************************/
  bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                       Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u);

   /** Get the starting point for the nonlinear problem to be solved with ipopt
   * @param n is the dimension of the variable
   * @param init_x is the starting point of the optimization problem
   * @param x is the variable
   * @param init_z is an ipopt variable
   * @param z_L is an ipopt variable
   * @param z_U is an ipopt variable
   * @param m is the number of constraints
   * @param init_lambda is an ipopt variable
   * @param lambda is an ipopt variable
   * @return true
   */
   /****************************************************************/
   bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                               bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                               Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda);

    /** Cost function of the nonlinear problem to be solved with ipopt
    * @param n is the dimension of the variable
    * @param x is the variable
    * @param new_x takes into account is the variable has been updated or not
    * @param obj_value is the value of the cost function
    * @retun true
    */
    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                   Ipopt::Number &obj_value);

    /** Auxiliary function for computing cost function of the nonlinear problem to be solved with ipopt
    * @param x is the variable
    * @param points_on is object point cloud
    * @param new_x takes into account is the variable has been updated or not
    * @retun the cost function value
    */
    /****************************************************************/
    void F(const Ipopt::Number *x, bool &new_x);

    /** Auxiliary function for computing cost function of the nonlinear problem to be solved with ipopt
    * @param obj is the Vector of the object
    * @param x is the variable
    * @param point is one point of the point cloud
    * @return a part of the cost function value
    */
    /****************************************************************/
    double f(const Ipopt::Number *x, const Eigen::Vector3d &point);

    /** Auxiliary function for computing the gradient of cost function of the nonlinear problem
    * @param x is the variable
    * @param points_on is one point of object point cloud
    * @return cost function value
    */
    /****************************************************************/
    double F_v(const Eigen::VectorXd &x);

    /** Auxiliary function for computing the gradient cost function of the nonlinear problem
    * @param obj is the Vector of the object
    * @param x is the variable
    * @param point is one point of the point cloud
    * @return a part of the cost function value
    */
    /****************************************************************/
    double f_v(const Eigen::VectorXd &x, const Eigen::Vector3d &point);

    /** Gradient of the cost function of the nonlinear problem
    * @param x is the variable
    * @param n is the dimension of the variable
    * @param new_x takes into account is the variable has been updated or not
    * @param grad_f is the gradient of the cost function
    */
    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f);

    /** Constraints of the nonlinear problem
    * @param n is the dimension of the variable
    * @param x is the variable
    * @param m is the number of constraints
    * @param new_x takes into account is the variable has been updated or not
    * @param g is the values of the constraints
    * @return true
    */
    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g);

    /**  Jacobian of the constraints of the nonlinear problem
    * @param n is the dimension of the variable
    * @param x is the variable
    * @param m is the number of constraints
    * @param new_x takes into account is the variable has been updated or not
    * @param iRow contains the jacobian raws
    * @param iCol contains the jacobian columns
    * @param values contains the jacobian values
    * @return true
    */
    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values);


    /** Compute a good starting point for the nonlinear problem
    * @param x0 is the initial value for the superquadric to be estimated
    * @param point_cloud is the object point cloud
    */
    /****************************************************************/
    void computeX0(Eigen::VectorXd &x0);

    /** Finalize the solution
    * @param n is the dimension of the variable
    * @param x is the variable
    * @param m is the number of constraints
    * @param init_z is an ipopt variable
    * @param z_L is an ipopt variable
    * @param z_U is an ipopt variable
    * @param status says if the problem has been solved or not
    * @param obj_value is the final cost function values
    */
    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                           const Ipopt::Number *x, const Ipopt::Number *z_L,
                           const Ipopt::Number *z_U, Ipopt::Index m,
                           const Ipopt::Number *g, const Ipopt::Number *lambda,
                           Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                           Ipopt::IpoptCalculatedQuantities *ip_cq);
public:
  SuperqModel::Superquadric solution;
  SuperqModel::PointCloud points_downsampled;

  /** Init function */
  /****************************************************************/
  void init();

  /** Set point to be used for superquadric estimation
  * @param point_cloud is the object point cloud
  * @param optimizer_points is the maximum number of points to be used for the optimization problem
  */
  /****************************************************************/
  void setPoints(SuperqModel::PointCloud &point_cloud, const int &optimizer_points);

  /** Configure function
  * @param rf is the resource finder
  * @param bounds_aut is to set or not the automatic computation of the variable bound
  * @param object_class is the object class according to its shape
  */
  /****************************************************************/
  void configure(std::string object_class);

  /** Extract the solution
  * @return the superquadric as a Vector
  */
  /****************************************************************/
  SuperqModel::Superquadric get_result() const;

  /** Compute superquadric solution
  /* @param Pointcloud
  * @return the superquadric solution
  */
  /****************************************************************/
  SuperqModel::Superquadric computeSuperq(PointCloud &pc);

};

struct IpoptParam
{
    double tol;
    int acceptable_iter;
    std::string mu_strategy;
    int max_iter;
    double max_cpu_time;
    std::string nlp_scaling_method;
    std::string hessian_approximation;
    int print_level;
    std::string object_class;
    int optimizer_points;
};


class EstimatorApp
{
public:
    SuperqModel::Superquadric computeSuperq(SuperqModel::IpoptParam &pars, PointCloud &point_cloud);
};


}

#endif
