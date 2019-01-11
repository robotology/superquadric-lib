#include <cmath>
#include <limits>
#include <iomanip>
#include <boost/range/irange.hpp>

#include "graspComputation.h"

using namespace std;
using namespace Eigen;
using namespace boost;
using namespace SuperqModel;
using namespace SuperqGrasp;


/****************************************************************/
void graspComputation::init(Superquadric &object_superq, Superquadric &hand_superq, const deque<Superquadric> &obstacles_superq, int &n_handpoints, const string &str_hand)
{
    hand = hand_superq.getSuperqParams();
    object = object_superq.getSuperqParams();

    for (size_t i=0; i<obstacles_superq.size(); i++)
    {
        Superquadric obst=obstacles_superq[i];
        obstacles.push_back(obst.getSuperqParams());
    }

    if (obstacles_superq.size()!= num_superq)
        num_superq = obstacles_superq.size();


    euler_obj = object_superq.getSuperqEulerZYZ();


    Matrix3d R;
    R = AngleAxisd(euler_obj(0), Vector3d::UnitZ())*         // To make it more efficient
        AngleAxisd(euler_obj(1), Vector3d::UnitY())*
        AngleAxisd(euler_obj(2), Vector3d::UnitZ());

    H_o2w.block(0,0,3,3) = R;

    H_o2w.block(0,3,3,1)=object_superq.getSuperqCenter();
    H_o2w.transposeInPlace();

    euler_hand = hand_superq.getSuperqEulerZYZ();
    R = AngleAxisd(euler_hand(0), Vector3d::UnitZ())*         // To make it more efficient
        AngleAxisd(euler_hand(1), Vector3d::UnitY())*
        AngleAxisd(euler_hand(2), Vector3d::UnitZ());

    H_h2w.block(0,0,3,3) = R;
    H_h2w.block(0,3,3,1)=hand_superq.getSuperqCenter();

    for (auto i: irange(0, (int)sqrt(n_handpoints), 1))
    {
        for (double theta=0; theta < 2*M_PI; theta+= M_PI/((int)sqrt(n_handpoints)))
        {
            Vector3d point=computePointsHand(hand,i, (int)sqrt(n_handpoints), str_hand, theta);
            Vector4d point_tr;

            if (str_hand=="right")
            {
                if (point[0] + point[2] <= 0)
                {
                    Vector4d point_tmp;
                    point_tmp.segment(0,3)=point;
                    point_tmp(3)=1;
                    point_tr=H_h2w*point_tmp;
                    point=point_tr.segment(0,3);
                    points_on.push_back(point);
                }
            }
            else
            {
                if (point[0] - point[2] < 0)
                {
                    Vector4d point_tmp;
                    point_tmp.segment(0,3)=point;
                    point_tmp(3)=1;
                    point_tr=H_h2w*point_tmp;
                    point=point_tr.segment(0,3);
                    points_on.push_back(point);
                }
            }

        }
    }

    aux_objvalue=0.0;
}

/****************************************************************/
Vector3d graspComputation::computePointsHand(Vector11d &hand, int j, int l, const string &str_hand, double &theta)
{
    Vector3d point(3);
    double omega;
    double ce,se,co,so;

    if (object.segment(0,3).maxCoeff()> object.segment(0,3).maxCoeff())
        hand(1)=object.segment(0,3).maxCoeff();

    omega=j*M_PI/(l);

    se=sin(theta);
    ce=cos(theta);
    co=cos(omega);
    so=sin(omega);

    point(0)=hand(0) * sign(ce)*(pow(abs(ce),hand(3))) * sign(co)*(pow(abs(co),hand(4)));
    point(1)=hand(1) * sign(se)*(pow(abs(se),hand(3)));
    point(2)=hand(2) * sign(ce)*(pow(abs(ce),hand(3))) * sign(so)*(pow(abs(so),hand(4)));

    return point;
}

/****************************************************************/
double graspComputation::sign(double &v)
{
  return ((v==0.0)?0.0:((v>0.0)?1.0:-1.0));
}

/****************************************************************/
bool graspComputation::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                                  Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style)
{
    n=6;
    if (num_superq==0)
        m=5;
    else
        m=5+(num_superq);

    nnz_jac_g=n*m;
    nnz_h_lag=0;
    index_style=TNLP::C_STYLE;
    bounds.resize(n,2);
    bounds_constr.resize(m,2);

    return true;
}

/****************************************************************/
bool graspComputation::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                                      Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    for (Ipopt::Index i=0; i<n; i++)
    {
       x_l[i]=bounds(i,0);
       x_u[i]=bounds(i,1);
    }

    for (Ipopt::Index i=0; i<m; i++)
    {
       g_l[i]=bounds_constr(i,0);
       g_u[i]=bounds_constr(i,1);
    }

    return true;
}

/****************************************************************/
 bool graspComputation::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                          bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                                          Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
 {
     for(Ipopt::Index i=0;i<n;i++)
     {
         x[i]=hand(i+5);
     }

     return true;
 }

 /****************************************************************/
 bool graspComputation::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                              Ipopt::Number &obj_value)
 {
     F(x,points_on,new_x);
     obj_value=aux_objvalue;

     return true;
 }

 /****************************************************************/
 double graspComputation::F(const Ipopt::Number *x, deque<Vector3d> &points_on, bool new_x)
 {
     double value=0.0;

     for(auto point : points_on)
     {
         value+= pow( pow(f(x,point),object(3))-1,2 );
     }

     value*=object(0)*object(1)*object(2)/points_on.size();

     aux_objvalue=value;
 }

 /****************************************************************/
 double graspComputation::f(const Ipopt::Number *x, Vector3d &point)
 {
     Vector4d point_tr;
     Vector4d point_tmp;
     point_tmp(3)=1;
     point_tmp.segment(0,3)=point;

     euler_hand(0)=x[3];
     euler_hand(1)=x[4];
     euler_hand(2)=x[5];


     Matrix3d R;
     R = AngleAxisd(euler_hand(0), Vector3d::UnitZ())*         // To make it more efficient
         AngleAxisd(euler_hand(1), Vector3d::UnitY())*
         AngleAxisd(euler_hand(2), Vector3d::UnitZ());

     H_x.block(0,0,3,3) = R;
     H_x(0,3)=x[0];
     H_x(1,3)=x[1];
     H_x(2,3)=x[2];

     point_tr.noalias()=H_x*point_tmp;

     double num1=H_o2w(0,0)*point_tr(0)+H_o2w(0,1)*point_tr(1)+H_o2w(0,2)*point_tr(2)-object(5)*H_o2w(0,0)-object(6)*H_o2w(0,1)-object(7)*H_o2w(0,2);
     double num2=H_o2w(1,0)*point_tr(0)+H_o2w(1,1)*point_tr(1)+H_o2w(1,2)*point_tr(2)-object(5)*H_o2w(1,0)-object(6)*H_o2w(1,1)-object(7)*H_o2w(1,2);
     double num3=H_o2w(2,0)*point_tr(0)+H_o2w(2,1)*point_tr(1)+H_o2w(2,2)*point_tr(2)-object(5)*H_o2w(2,0)-object(6)*H_o2w(2,1)-object(7)*H_o2w(2,2);

     double tmp=pow(abs(num1/object(0)),2.0/object(4)) + pow(abs(num2/object(1)),2.0/object(4));

     return pow( abs(tmp),object(4)/object(3)) + pow( abs(num3/object(2)),(2.0/object(3)));
 }

 /****************************************************************/
 double graspComputation::F_v(Vector6d &x, deque<Vector3d> &points_on)
 {
     double value=0.0;

     for(auto point : points_on)
        value+= pow( pow(f_v(x,point),object(3))-1,2 );

     value*=object(0)*object(1)*object(2)/points_on.size();

     return value;
 }

/****************************************************************/
 double graspComputation::f_v(Vector6d &x, Vector3d &point)
 {
     Vector4d point_tr;
     Vector4d point_tmp;
     point_tmp(3)=1;
     point_tmp.segment(0,3)=point;

     Matrix3d R;
     R = AngleAxisd(x(3), Vector3d::UnitZ())*         // To make it more efficient
         AngleAxisd(x(4), Vector3d::UnitY())*
         AngleAxisd(x(5), Vector3d::UnitZ());

     H_x.block(0,0,3,3) = R;
     H_x(0,3)=x[0];
     H_x(1,3)=x[1];
     H_x(2,3)=x[2];

     point_tr.noalias()=H_x*point_tmp;

     double num1=H_o2w(0,0)*point_tr(0)+H_o2w(0,1)*point_tr(1)+H_o2w(0,2)*point_tr(2)-object(5)*H_o2w(0,0)-object(6)*H_o2w(0,1)-object(7)*H_o2w(0,2);
     double num2=H_o2w(1,0)*point_tr(0)+H_o2w(1,1)*point_tr(1)+H_o2w(1,2)*point_tr(2)-object(5)*H_o2w(1,0)-object(6)*H_o2w(1,1)-object(7)*H_o2w(1,2);
     double num3=H_o2w(2,0)*point_tr(0)+H_o2w(2,1)*point_tr(1)+H_o2w(2,2)*point_tr(2)-object(5)*H_o2w(2,0)-object(6)*H_o2w(2,1)-object(7)*H_o2w(2,2);

     double tmp=pow(abs(num1/object(0)),2.0/object(4)) + pow(abs(num2/object(1)),2.0/object(4));

     return pow( abs(tmp),object(4)/object(3)) + pow( abs(num3/object(2)),(2.0/object(3)));
 }

 /****************************************************************/
 bool graspComputation::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number *grad_f)
 {
     Vector6d x_tmp;
     double grad_p, grad_n;
     double eps=1e-6;

     for(Ipopt::Index i=0;i<n;i++)
        x_tmp(i)=x[i];

     for(Ipopt::Index j=0;j<n;j++)
     {
         x_tmp(j)=x_tmp(j)+eps;

         grad_p=F_v(x_tmp,points_on);

         x_tmp(j)=x_tmp(j)-eps;

         grad_n=F_v(x_tmp,points_on);

         grad_f[j]=(grad_p-grad_n)/eps;
     }

     return true;
 }


/*****************************************************************/
GraspPoses GraspEstimatorApp::computeGraspPoses(IpoptParam &pars, Superquadric)
{

}
