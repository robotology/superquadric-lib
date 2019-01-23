/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>

#include "superquadricEstimator.h"

using namespace std;
using namespace Eigen;
using namespace SuperqModel;

/****************************************************************/
void SuperqEstimator::init()
{
    points_downsampled.deletePoints();
    aux_objvalue=0.0;
}

/****************************************************************/
void SuperqEstimator::setPoints(PointCloud &point_cloud, const int &optimizer_points, bool &random)
{
    // Set points from point cloud and downsample them if too many
    if (point_cloud.getNumberPoints()>optimizer_points)
        point_cloud.subSample(optimizer_points, random);

    points_downsampled=point_cloud;

    used_points=points_downsampled.getNumberPoints();

    cout<<"[SuperquadricEstimator]: points actually used for modeling: "<<used_points<<endl;

    x0.resize(11);
    x0.setZero();
    // Compute initial estimate of superquadric
    computeX0(x0, point_cloud);
}

/****************************************************************/
void SuperqEstimator::computeX0(Vector11d &x0, PointCloud &point_cloud)
{
    // Exponents are computed according to object class, defined in configure
    x0(3)=(bounds(3,0)+bounds(3,1))/2;
    x0(4)=(bounds(4,0)+bounds(4,1))/2;
    x0(5)=x0(6)=x0(7)=0.0;

    // Initial orientation is obtained from point cloud
    Matrix3d orientation;
    orientation=point_cloud.getAxes();
    x0.segment(8,3)=orientation.eulerAngles(2,1,2);

    // Initial value for superquadric dimensions is obtained from point cloud
    // bounding boxes
    Matrix32d bounding_box(3,2);
    bounding_box=point_cloud.getBoundingBox();
    x0(0)=(-bounding_box(0,0)+bounding_box(0,1))/2;
    x0(1)=(-bounding_box(1,0)+bounding_box(1,1))/2;
    x0(2)=(-bounding_box(2,0)+bounding_box(2,1))/2;

    // Intial value of the superquadric center is obtained from the point cloud
    bounding_box = orientation * bounding_box;
    x0(5) = (bounding_box(0,0)+bounding_box(0,1))/2;
    x0(6) = (bounding_box(1,0)+bounding_box(1,1))/2;
    x0(7) = (bounding_box(2,0)+bounding_box(2,1))/2;
}

/****************************************************************/
bool SuperqEstimator::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                  Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style)
{
    // Number of variable to estimate is 11, the parameters of the object superquadric
    n=11;
    m=nnz_jac_g=nnz_h_lag=0;
    index_style=TNLP::C_STYLE;

    return true;
}

/****************************************************************/
void SuperqEstimator::computeBounds()
{
    // Compute bounds starting from x0
    bounds(0,1)=x0(0)*1.3;
    bounds(1,1)=x0(1)*1.3;
    bounds(2,1)=x0(2)*1.3;

    bounds(0,0)=x0(0)*0.7;
    bounds(1,0)=x0(1)*0.7;
    bounds(2,0)=x0(2)*0.7;

    bounds(5,0)=x0(5)-bounds(0,1);
    bounds(6,0)=x0(6)-bounds(1,1);
    bounds(7,0)=x0(7)-bounds(2,1);
    bounds(5,1)=x0(5)+bounds(0,1);
    bounds(6,1)=x0(6)+bounds(1,1);
    bounds(7,1)=x0(7)+bounds(2,1);

    bounds(8,0)=0;
    bounds(9,0)=0;
    bounds(10,0)=0;
    bounds(8,1)=2*M_PI;
    bounds(9,1)=M_PI;
    bounds(10,1)=2*M_PI;
}

/****************************************************************/
bool SuperqEstimator::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                     Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    computeBounds();

    for (Ipopt::Index i=0; i<n; i++)
    {
       x_l[i]=bounds(i,0);
       x_u[i]=bounds(i,1);
    }

    return true;
}

/****************************************************************/
 bool SuperqEstimator::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
 {
     // Set initial x
     for (Ipopt::Index i=0;i<n;i++)
     {
         x[i]=x0[i];
     }

     return true;
 }

 /****************************************************************/
  bool SuperqEstimator::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Number &obj_value)
  {
      // Compute cost function
      F(x, new_x);
      obj_value=aux_objvalue;

      return true;
  }

  /****************************************************************/
 void SuperqEstimator::F(const Ipopt::Number *x, bool &new_x)
 {
     if (new_x)
     {
         double value=0.0;
         for(auto point : points_downsampled.points)
         {
             double tmp=pow(f(x,point),x[3])-1;
             value+=tmp*tmp;
         }
         value*=x[0]*x[1]*x[2]/used_points;
         aux_objvalue=value;
     }
 }

 /****************************************************************/
 double SuperqEstimator::f(const Ipopt::Number *x, const Vector3d &point_cloud)
 {
     Vector3d euler;
     euler(0)=x[8];
     euler(1)=x[9];
     euler(2)=x[10];
     Matrix3d R;
     R = AngleAxisd(euler(0), Vector3d::UnitZ())*
         AngleAxisd(euler(1), Vector3d::UnitY())*
         AngleAxisd(euler(2), Vector3d::UnitZ());

     double num1=R(0,0)*point_cloud(0)+R(1,0)*point_cloud(1)+R(2,0)*point_cloud(2)-x[5]*R(0,0)-x[6]*R(1,0)-x[7]*R(2,0);
     double num2=R(0,1)*point_cloud(0)+R(1,1)*point_cloud(1)+R(2,1)*point_cloud(2)-x[5]*R(0,1)-x[6]*R(1,1)-x[7]*R(2,1);
     double num3=R(0,2)*point_cloud(0)+R(1,2)*point_cloud(1)+R(2,2)*point_cloud(2)-x[5]*R(0,2)-x[6]*R(1,2)-x[7]*R(2,2);
     double tmp=pow(abs(num1/x[0]),2.0/x[4]) + pow(abs(num2/x[1]),2.0/x[4]);

     return pow( abs(tmp),x[4]/x[3]) + pow( abs(num3/x[2]),(2.0/x[3]));
 }

 /****************************************************************/
 double SuperqEstimator::F_v(const Vector11d &x)
 {
     // Evaluate cost function for finite difference gradient computation
     double value=0.0;

     for(auto point : points_downsampled.points)
     {
          double tmp=pow(f_v(x,point),x(3))-1;
          value+=tmp*tmp;
     }

     value*=x(0)*x(1)*x(2)/used_points;

     return value;
 }

   /****************************************************************/
  double SuperqEstimator::f_v(const Vector11d &x, const Vector3d &point_cloud)
  {
      Matrix3d R;
      R = AngleAxisd(x(8), Vector3d::UnitZ())*
          AngleAxisd(x(9), Vector3d::UnitY())*
          AngleAxisd(x(10), Vector3d::UnitZ());

      double num1=R(0,0)*point_cloud(0)+R(1,0)*point_cloud(1)+R(2,0)*point_cloud(2)-x(5)*R(0,0)-x(6)*R(1,0)-x(7)*R(2,0);
      double num2=R(0,1)*point_cloud(0)+R(1,1)*point_cloud(1)+R(2,1)*point_cloud(2)-x(5)*R(0,1)-x(6)*R(1,1)-x(7)*R(2,1);
      double num3=R(0,2)*point_cloud(0)+R(1,2)*point_cloud(1)+R(2,2)*point_cloud(2)-x(5)*R(0,2)-x(6)*R(1,2)-x(7)*R(2,2);
      double tmp=pow(abs(num1/x(0)),2.0/x(4)) + pow(abs(num2/x(1)),2.0/x(4));

      return pow( abs(tmp),x(4)/x(3)) + pow( abs(num3/x(2)),(2.0/x(3)));
  }

  /****************************************************************/
 bool SuperqEstimator::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                  Ipopt::Number *grad_f)
 {
     // Evaluate gradient with finite differences
     Vector11d x_tmp(n);
     double grad_p, grad_n;
     double eps=1e-8;

     for (Ipopt::Index j=0;j<n;j++)
         x_tmp(j)=x[j];

     for (Ipopt::Index j=0;j<n;j++)
     {
         x_tmp(j)+=eps;

         grad_p=F_v(x_tmp);

         x_tmp(j)-=eps;

         grad_n=F_v(x_tmp);

         grad_f[j]=(grad_p-grad_n)/eps;
      }

     return true;
 }

 /****************************************************************/
 bool SuperqEstimator::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
             Ipopt::Index m, Ipopt::Number *g)
 {
     // No constraints in this problem
     return false;
 }

 /****************************************************************/
 bool SuperqEstimator::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                 Ipopt::Index *jCol, Ipopt::Number *values)
 {
     return false;
 }

 /****************************************************************/
void SuperqEstimator::configure(string object_class)
{
    // Configure exponents according to object class
    bounds.resize(11,2);

    obj_class=object_class;

    if (obj_class=="default")
    {
      bounds(3,0)=0.1;
      bounds(3,1)=1.9;
      bounds(4,0)=0.1;
      bounds(4,1)=1.9;
    }
    else if (obj_class=="box")
    {
      bounds(3,0)=0.01;
      bounds(3,1)=0.3;
      bounds(4,0)=0.01;
      bounds(4,1)=0.3;
    }
    else if (obj_class=="cylinder")
    {
      bounds(3,0)=0.01;
      bounds(3,1)=0.3;
      bounds(4,0)=0.8;
      bounds(4,1)=1.2;
    }
    else if (obj_class=="sphere")
    {
      bounds(3,0)=1.;
      bounds(3,1)=1.;
      bounds(4,0)=1.;
      bounds(4,1)=1.;
    }

}

/****************************************************************/
void SuperqEstimator::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                      const Ipopt::Number *x, const Ipopt::Number *z_L,
                      const Ipopt::Number *z_U, Ipopt::Index m,
                      const Ipopt::Number *g, const Ipopt::Number *lambda,
                      Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                      Ipopt::IpoptCalculatedQuantities *ip_cq)
{
    // Save solution in Superquadric class
    Vector11d params_sol(n);
    for (Ipopt::Index i=0; i<n; i++)
        params_sol[i]=x[i];

    solution.setSuperqParams(params_sol);
}

/****************************************************************/
Superquadric SuperqEstimator::get_result() const
{
    return solution;
}

/****************************************************************/
Superquadric SuperqEstimatorApp::computeSuperq(IpoptParam &pars, PointCloud &point_cloud)
{
    // Process for estimate the superquadric
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",pars.tol);
    app->Options()->SetIntegerValue("acceptable_iter",pars.acceptable_iter);
    app->Options()->SetStringValue("mu_strategy",pars.mu_strategy);
    app->Options()->SetIntegerValue("max_iter",pars.max_iter);
    app->Options()->SetNumericValue("max_cpu_time",pars.max_cpu_time);
    app->Options()->SetStringValue("nlp_scaling_method",pars.nlp_scaling_method);
    app->Options()->SetStringValue("hessian_approximation",pars.hessian_approximation);
    app->Options()->SetIntegerValue("print_level",pars.print_level);
    //app->Options()->SetStringValue("derivative_test","first-order");
    app->Initialize();

    Ipopt::SmartPtr<SuperqEstimator> estim = new SuperqEstimator;
    estim->init();
    estim->configure(pars.object_class);
    estim->setPoints(point_cloud, pars.optimizer_points, pars.random_sampling);

    clock_t tStart = clock();

    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(estim));

    double computation_time=(double)(clock() - tStart)/CLOCKS_PER_SEC;

    Superquadric superq;

    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols,", ", ", ", "", "", " [ ", "]");

    if (status==Ipopt::Solve_Succeeded)
    {
        superq=estim->get_result();
        cout<<"|| ---------------------------------------------------- ||"<<endl;
        cout<<"|| Superquadric estimated                         : ";
        cout<<superq.getSuperqParams().format(CommaInitFmt)<<endl<<endl;
        cout<<"|| Computed in                                    :  ";
        cout<<   computation_time<<" [s]"<<endl;
        cout<<"|| ---------------------------------------------------- ||"<<endl<<endl<<endl;
        return superq;
    }
    else if(status==Ipopt::Maximum_CpuTime_Exceeded)
    {
        superq=estim->get_result();
        cout<<"|| ---------------------------------------------------- ||"<<endl;
        cout<<"|| Time expired                                         :"<<superq.getSuperqParams().format(CommaInitFmt)<<endl<<endl;
        cout<<"|| Superquadric estimated in                            :  "<<   computation_time<<" [s]"<<endl;
        cout<<"|| ---------------------------------------------------- ||"<<endl<<endl<<endl;
        return superq;
    }
    else
    {
        cout<<"|| ---------------------------------------------------- ||"<<endl;
        cerr<<"|| Not solution found"<<endl;
        cout<<"|| ---------------------------------------------------- ||"<<endl<<endl<<endl;
        Vector11d x(11);
        x.setZero();

        superq.setSuperqParams(x);
        return superq;
    }

}
