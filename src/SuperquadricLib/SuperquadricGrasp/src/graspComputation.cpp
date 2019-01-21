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
void graspComputation::init(GraspParams &g_params)
{
    hand = g_params.hand_superq.getSuperqParams();
    object = g_params.object_superq.getSuperqParams();
    n_hands = g_params.n_hands;
    l_o_r = g_params.left_or_right;

    for (size_t i=0; i<g_params.obstacles_superq.size(); i++)
    {
        Superquadric obst=g_params.obstacles_superq[i];
        obstacles.push_back(obst.getSuperqParams());
    }

    if (g_params.obstacles_superq.size()!= num_superq)
        num_superq = g_params.obstacles_superq.size();

    Vector3d euler_obj = g_params.object_superq.getSuperqEulerZYZ();

    Matrix3d R;
    R = AngleAxisd(euler_obj(0), Vector3d::UnitZ())*         // To make it more efficient
        AngleAxisd(euler_obj(1), Vector3d::UnitY())*
        AngleAxisd(euler_obj(2), Vector3d::UnitZ());

    H_o2w.block(0,0,3,3) = R;

    H_o2w.col(3).segment(3,1)=g_params.object_superq.getSuperqCenter();
    H_o2w.transposeInPlace();

    Vector3d euler_hand = g_params.hand_superq.getSuperqEulerZYZ();
    R = AngleAxisd(euler_hand(0), Vector3d::UnitZ())*         // To make it more efficient
        AngleAxisd(euler_hand(1), Vector3d::UnitY())*
        AngleAxisd(euler_hand(2), Vector3d::UnitZ());

    H_h2w.block(0,0,3,3) = R;
    H_h2w.col(3).segment(3,1)=g_params.hand_superq.getSuperqCenter();

    for (auto i: irange(0, (int)sqrt(n_hands), 1))
    {
        for (double theta=0; theta < 2*M_PI; theta+= M_PI/((int)sqrt(n_hands)))
        {
            Vector3d point=computePointsHand(hand,i, (int)sqrt(n_hands), l_o_r, theta);

            if (l_o_r=="right")
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

    // Configure parameters for constraints
    if (l_o_r=="right")
    {
        d_x(0)=-0.7; d_x(1)=-0.7; d_x(2)= -0.7;
        d_y(0)= 0.0; d_y(1)= 0.7; d_y(2)= -0.7;
        d_z(0)= 0.0; d_z(1)= -0.7; d_z(2)= -0.7;
    }
    else
    {
        d_x(0)=-0.7; d_x(1)=0.7; d_x(2)= -0.7;
        d_y(0)= 0.0; d_y(1)= -0.7; d_y(2)=-0.7;
        d_z(0)=0.0; d_z(1)=-0.7; d_z(2)= 0.7;
    }

    if (num_superq>0)
       theta_x=M_PI/4.0;
    else
        theta_x=M_PI/6.0;
    theta_y=M_PI/4.0;
    theta_z=M_PI/4.0;

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
 Matrix4d computeMatrix(const Vector6d &current_pose)
 {
     Matrix4d H_tmp;
     Matrix3d R;
     R = AngleAxisd(current_pose(3), Vector3d::UnitZ())*         // To make it more efficient
         AngleAxisd(current_pose(4), Vector3d::UnitY())*
         AngleAxisd(current_pose(5), Vector3d::UnitZ());

     H_tmp.block(0,0,3,3) = R;
     H_tmp(0,3)=current_pose(0);
     H_tmp(1,3)=current_pose(1);
     H_tmp(2,3)=current_pose(2);

     return H_tmp;
 }

 /****************************************************************/
 double graspComputation::f(const Ipopt::Number *x, Vector3d &point)
 {
     point_tmp(3)=1;
     point_tmp.segment(0,3)=point;

     for (auto i: irange(0,6,1))
        x_tmp(i)=x[i];

     H_x = computeMatrix(x_tmp);

     point_tr.noalias()=H_x*point_tmp;

     Vector3d point3=point_tr.segment(0,3);

     return f_v2(object,point3);
 }

 /****************************************************************/
 double graspComputation::F_v(Vector6d &x)
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
     point_tmp(3)=1;
     point_tmp.segment(0,3)=point;

     H_x = computeMatrix(x);

     point_tr.noalias()=H_x*point_tmp;

     Vector3d point3=point_tr.segment(0,3);

     return f_v2(object,point3);
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

         grad_p=F_v(x_tmp);

         x_tmp(j)=x_tmp(j)-eps;

         grad_n=F_v(x_tmp);

         grad_f[j]=(grad_p-grad_n)/eps;
     }

     return true;
 }

 /****************************************************************/
 double graspComputation::coneImplicitFunction(const Vector3d &point, const Vector3d &d, double theta)
 {
    double d_dot_d=d.dot(d);
    double p_dot_p=point.dot(point);
    double p_dot_d=point.dot(d);

    return  (-p_dot_d + cos(theta));
 }

 /****************************************************************/
 bool graspComputation::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
             Ipopt::Index m, Ipopt::Number *g)
 {
     for (auto i: irange(0,6,1))
        x_tmp(i)=x[i];

     H_x = computeMatrix(x_tmp);

     H.noalias()=H_x*H_h2w;

     d_x=d_x/d_x.norm();
     d_y=d_y/d_y.norm();
     d_z=d_z/d_z.norm();

     double F_x, F_y, F_z;

     Vector3d x_hand, y_hand, z_hand;

     x_hand=H.col(0).segment(3,1);

     y_hand=H.col(1).segment(3,1);

     z_hand=H.col(2).segment(3,1);

     F_x=coneImplicitFunction(x_hand, d_x, theta_x);
     F_y=coneImplicitFunction(y_hand, d_y, theta_y);
     F_z=coneImplicitFunction(z_hand, d_z, theta_z);

     g[0]= F_x;
     g[1]= F_y;
     g[2]= F_z;

     Vector3d x_min;
     double minz=10.0;

     for (auto point : points_on)
     {
         Vector4d pnt;
         pnt.segment(0,3)=point;
         pnt(3)=1;
         Vector4d p=H_x*pnt;

         if (p(2)<minz)
         {
             minz=p(2);
             x_min=p.segment(0,3);
         }
     }

     g[3]=plane(0,0)*x_min(0)+plane(1,0)*x_min(1)+plane(2,0)*x_min(2)+plane(3,0);

     Vector3d robotPose;

     if (l_o_r=="right")
     {
        robotPose=x_tmp.segment(0,3)-hand(0)*z_hand;
        robotPose = robotPose-hand(0)*x_hand;
     }
     else
     {
         robotPose=x_tmp.segment(0,3)+hand(0)*z_hand;
         robotPose = robotPose-hand(0)*x_hand;
     }

     g[4]=object(0)*object(1)*object(2)*(pow(f_v2(object,robotPose), object(3)) -1);


     if (num_superq>0)
     {
         for (auto j: irange(0,num_superq, 1))
         {
             g[5+j]=0;

             g[5+j]=computeObstacleValues(x,j);
         }
     }

     return true;
 }

 /****************************************************************/
 double graspComputation::G_v(Vector6d &x, int i, Ipopt::Index m)
 {
     H_x = computeMatrix(x);

     H.noalias()=H_x*H_h2w;

     d_x=d_x/d_x.norm();
     d_y=d_y/d_y.norm();
     d_z=d_z/d_z.norm();

     double F_x, F_y, F_z;

     Vector3d x_hand, y_hand, z_hand;

     x_hand=H.col(0).segment(3,1);

     y_hand=H.col(1).segment(3,1);

     z_hand=H.col(2).segment(3,1);

     F_x=coneImplicitFunction(x_hand, d_x, theta_x);
     F_y=coneImplicitFunction(y_hand, d_y, theta_y);
     F_z=coneImplicitFunction(z_hand, d_z, theta_z);

     g[0]= F_x;
     g[1]= F_y;
     g[2]= F_z;

     Vector3d x_min;
     double minz=10.0;

     for (auto point : points_on)
     {
         Vector4d pnt;
         pnt.segment(0,3)=point;
         pnt(3)=1;
         Vector4d p=H_x*pnt;

         if (p(2)<minz)
         {
             minz=p(2);
             x_min=p.segment(0,3);
         }
     }

     g[3]=plane(0,0)*x_min(0)+plane(1,0)*x_min(1)+plane(2,0)*x_min(2)+plane(3,0);

     Vector3d robotPose;

     if (l_o_r=="right")
     {
        robotPose=x.segment(0,3)-hand(0)*z_hand;
        robotPose = robotPose-hand(0)*x_hand;
     }
     else
     {
         robotPose=x.segment(0,3)+hand(0)*z_hand;
         robotPose = robotPose-hand(0)*x_hand;
     }

     g[4]=object(0)*object(1)*object(2)*(pow(f_v2(object,robotPose), object(3)) -1);


     if (num_superq>0)
     {
         for (auto j: irange(0,num_superq, 1))
         {
             g[5+j]=0;

             g[5+j]=computeObstacleValues_v(x,j);
         }
     }

     return g[i];
 }

 /****************************************************************/
 bool graspComputation::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                 Ipopt::Index *jCol, Ipopt::Number *values)
 {
     Vector6d x_tmp;
     double grad_p, grad_n;
     double eps=1e-6;

     if(values!=NULL)
     {
         for(Ipopt::Index i=0;i<n;i++)
            x_tmp(i)=x[i];

         int count=0;
         for(Ipopt::Index i=0;i<m; i++)
         {
             for(Ipopt::Index j=0;j<n;j++)
             {
                 x_tmp(j)=x_tmp[j]+eps;

                 grad_p=G_v(x_tmp,i,m);
                 x_tmp(j)=x_tmp[j]-eps;

                 grad_n=G_v(x_tmp,i,m);

                 values[count]=(grad_p-grad_n)/(eps);
                 count++;
             }
         }
     }
     else
    {
        if (num_superq==0)
        {
            for (auto j: irange(0, m, 1))
            {
                for (auto i: irange(0, n, 1))
                {
                    jCol[j*(n) + i]= i;
                    iRow[j*(n)+ i] = j;
                }
            }
        }
        else
        {
            for (auto j: irange(0, m, 1))
            {
                for (auto i: irange(0, n, 1))
                {
                    jCol[j*(n) + i]= i;
                    iRow[j*(n)+ i] = j;
                }
            }
        }
     }

     return true;

 }

 /****************************************************************/
void graspComputation::configure(GraspParams &g_params)
{
    l_o_r = g_params.left_or_right;

    if (l_o_r=="right")
        bounds = g_params.bounds_right;
    else if (l_o_r=="left")
        bounds = g_params.bounds_left;

    max_superq = g_params.max_superq;

    bounds_constr.resize(5 + max_superq -1, 2);
    if (l_o_r=="right")
        bounds_constr = g_params.bounds_constr_right;
    else if (l_o_r=="left")
        bounds_constr = g_params.bounds_constr_left;

    displacement=g_params.disp;
    plane=g_params.pl;

    g.resize(5+max_superq - 1);
}

/****************************************************************/
void graspComputation::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                      const Ipopt::Number *x, const Ipopt::Number *z_L,
                      const Ipopt::Number *z_U, Ipopt::Index m,
                      const Ipopt::Number *g, const Ipopt::Number *lambda,
                      Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                      Ipopt::IpoptCalculatedQuantities *ip_cq)
{
     for (auto i: irange(0,6,1))
         solution_vector(i)=x[i];

     H_x = computeMatrix(solution_vector);

     if (notAlignedPose(H_x))
         alignPose(H_x);

     Matrix3d R = H_x.block(0,0,3,3);

     solution_vector.segment(3,3)= R.eulerAngles(2,1,2);

     for (Ipopt::Index i=0; i<3; i++)
         solution_vector(i)=H_x(i,3);

      robot_pose = solution_vector;

      if (l_o_r=="right")
      {
          robot_pose.segment(0,3) = solution_vector.segment(0,3)-(hand(0)+displacement(2))*(H_x.col(2).segment(3,1));
          robot_pose.segment(0,3).noalias() = robot_pose.segment(0,3)-(hand(0) + displacement(0))*(H_x.col(0).segment(3,1));
          robot_pose.segment(0,3).noalias() = robot_pose.segment(0,3)-(displacement(1))*(H_x.col(1).segment(3,1));
      }
      else
      {
          robot_pose.segment(0,3) = solution_vector.segment(0,3)+(hand(0)+displacement(2))*(H_x.col(2).segment(3,1));
          robot_pose.segment(0,3).noalias() = robot_pose.segment(0,3)-(hand(0) + displacement(0))*(H_x.col(0).segment(3,1));
          robot_pose.segment(0,3).noalias() = robot_pose.segment(0,3)-(displacement(1))*(H_x.col(1).segment(3,1));
      }

      final_F_value=0.0;

      for(auto point : points_on)
      {
          final_F_value += pow( pow(f_v(solution_vector,point),object(3))-1,2 );
      }

      final_F_value/=points_on.size();

      Vector11d x_aux;

      for ( auto i: irange(0,11,1))
          x_aux(i)=x[i];

      final_obstacles_value=computeFinalObstacleValues(robot_pose);

      solution.setGraspParams(solution_vector);
}

/****************************************************************/
GraspPoses graspComputation::get_result() const
{
   return solution;
}

/****************************************************************/
Superquadric graspComputation::get_hand() const
{
   Superquadric hand_superq;
   hand_superq.setSuperqParams(hand);
   return hand_superq;
}

/****************************************************************/
double graspComputation::get_final_F() const
{
   return final_F_value;
}

/****************************************************************/
deque<double> graspComputation::get_final_constr_values() const
{
   return final_obstacles_value;
}

/****************************************************************/
double graspComputation::computeObstacleValues(const Ipopt::Number *x, int k)
{
    deque<double> values;
    double constr_value=0.0;

    Vector3d middle_finger;
    Vector3d thumb_finger;
    Vector3d palm_up;
    Vector3d palm_down;

    deque<Vector3d> edges_hand;

    Vector6d pose_hand;

    for (auto i : irange(0,6,1))
        pose_hand(i)=x[i];

    Matrix4d H_robot;

    H_robot = computeMatrix(pose_hand);

    middle_finger=pose_hand.segment(0,3) - hand(0) * H_robot.col(0).segment(3,1);
    if (l_o_r=="right")
        thumb_finger=pose_hand.segment(0,3) + hand(2) * H_robot.col(2).segment(3,1);
    else
        thumb_finger=pose_hand.segment(0,3) - hand(2) * H_robot.col(2).segment(3,1);

    palm_up=pose_hand.segment(0,3) - hand(0) * H_robot.col(1).segment(3,1);
    palm_down=pose_hand.segment(0,3) + hand(0) * H_robot.col(1).segment(3,1);

    edges_hand.push_back(pose_hand.segment(0,3));
    edges_hand.push_back(middle_finger);
    edges_hand.push_back(thumb_finger);
    edges_hand.push_back(palm_up);
    edges_hand.push_back(palm_down);


    constr_value=0.0;

    Vector11d obstacle=obstacles[k];

    for (auto edge : edges_hand)
    {
        constr_value+=  pow(f_v2(obstacle,edge),obstacle[3])-1;
    }

    constr_value*=obstacle(0) * obstacle(1) * obstacle(2) /edges_hand.size();

    return constr_value;
}

/****************************************************************/
double graspComputation::computeObstacleValues_v(Vector6d &pose_hand, int k)
{
    Matrix4d H_robot;
    Vector3d middle_finger;
    Vector3d thumb_finger;
    Vector3d palm_up;
    Vector3d palm_down;

    double constr_value=0.0;

    deque<Vector3d> edges_hand;

    H_robot = computeMatrix(pose_hand);

    middle_finger=pose_hand.segment(0,3) - hand(0) * H_robot.col(0).segment(3,1);
    if (l_o_r=="right")
        thumb_finger=pose_hand.segment(0,3) + hand(2) * H_robot.col(2).segment(3,1);
    else
        thumb_finger=pose_hand.segment(0,3) - hand(2) * H_robot.col(2).segment(3,1);

    palm_up=pose_hand.segment(0,3) - hand(0) * H_robot.col(1).segment(3,1);
    palm_down=pose_hand.segment(0,3) + hand(0) * H_robot.col(1).segment(3,1);

    edges_hand.push_back(pose_hand.segment(0,3));
    edges_hand.push_back(middle_finger);
    edges_hand.push_back(thumb_finger);
    edges_hand.push_back(palm_up);
    edges_hand.push_back(palm_down);

    constr_value=0.0;

    Vector11d obstacle=obstacles[k];

    for (auto edge : edges_hand)
    {
        constr_value+=  pow(f_v2(obstacle,edge),obstacle[3])-1;
    }

    constr_value*=obstacle(0) * obstacle(1) * obstacle(2) /edges_hand.size();

    return constr_value;
}

/****************************************************************/
deque<double> graspComputation::computeFinalObstacleValues(Vector6d &pose_hand)
{
    deque<double> values;
    Vector3d middle_finger;
    Vector3d thumb_finger;
    Vector3d palm_up;
    Vector3d palm_down;
    Matrix4d H_robot;

    deque<Vector3d> edges_hand;
    double constr_value=0.0;

    H_robot = computeMatrix(pose_hand);

    middle_finger=pose_hand.segment(0,3) - hand(0) * H_robot.col(0).segment(3,1);
    if (l_o_r=="right")
        thumb_finger=pose_hand.segment(0,3) + hand(2) * H_robot.col(2).segment(3,1);
    else
        thumb_finger=pose_hand.segment(0,3) - hand(2) * H_robot.col(2).segment(3,1);

    palm_up=pose_hand.segment(0,3) - hand(0) * H_robot.col(1).segment(3,1);
    palm_down=pose_hand.segment(0,3) + hand(0) * H_robot.col(1).segment(3,1);

    edges_hand.push_back(pose_hand.segment(0,3));
    edges_hand.push_back(middle_finger);
    edges_hand.push_back(thumb_finger);
    edges_hand.push_back(palm_up);
    edges_hand.push_back(palm_down);

    for (auto obstacle : obstacles)
    {
        constr_value=0.0;

        for (auto edge: edges_hand)
        {
            constr_value+=  pow(f_v2(obstacle,edge),obstacle[3])-1;
        }

        constr_value*=obstacle(0) * obstacle(1) * obstacle(2) /points_on.size();

        values.push_back(constr_value);
    }

    return values;

}

/****************************************************************/
bool graspComputation::notAlignedPose(Matrix4d &final_H)
{
    if ((final_H(2,1)< 0.0 && final_H(2,1) > -0.5) ||  (final_H(2,1) < -0.8 && final_H(2,1) > -1.0))
    {
        if ((final_H(2,1)< 0.0 && final_H(2,1) > -0.5))
            top_grasp=true;
        else
            top_grasp=false;

        return true;
    }
    else
         return false;
}

/****************************************************************/
void graspComputation::alignPose(Matrix4d &final_H)
{
    Matrix3d rot_x;
    rot_x.setIdentity();

    Vector4d axis;
    double theta;

    if (top_grasp)
    {
        if (l_o_r=="right")
            theta=M_PI/2 - acos(-final_H(2,1));
        else
            theta=-(M_PI/2 - acos(-final_H(2,1)));

        rot_x(1,1)=rot_x(2,2)=cos(theta);
        rot_x(2,1)=sin(theta);
        rot_x(1,2) = -rot_x(2,1);

    }
    else
    {
        Vector3d z_axis;
        z_axis(2)=-1;

        Vector3d colum_1=final_H.col(1).segment(0,3);

        Vector3d cross_prod=colum_1.cross(z_axis);

        axis.segment(0,3)=cross_prod/cross_prod.norm();

        axis(3)=acos(colum_1.dot(z_axis)/(colum_1.norm()));

        rot_x=AngleAxisd(axis(3), axis.segment(0,3));
    }

    final_H.block(0,0,3,3).noalias()= rot_x * final_H.block(0,0,3,3);

}

/*****************************************************************/
double graspComputation::f_v2(Vector11d &obj, Vector3d &point_tr)
{
    double num1=H_o2w(0,0)*point_tr(0)+H_o2w(0,1)*point_tr(1)+H_o2w(0,2)*point_tr(2)-object(5)*H_o2w(0,0)-object(6)*H_o2w(0,1)-object(7)*H_o2w(0,2);
    double num2=H_o2w(1,0)*point_tr(0)+H_o2w(1,1)*point_tr(1)+H_o2w(1,2)*point_tr(2)-object(5)*H_o2w(1,0)-object(6)*H_o2w(1,1)-object(7)*H_o2w(1,2);
    double num3=H_o2w(2,0)*point_tr(0)+H_o2w(2,1)*point_tr(1)+H_o2w(2,2)*point_tr(2)-object(5)*H_o2w(2,0)-object(6)*H_o2w(2,1)-object(7)*H_o2w(2,2);

    double tmp=pow(abs(num1/object(0)),2.0/object(4)) + pow(abs(num2/object(1)),2.0/object(4));

    return pow( abs(tmp),object(4)/object(3)) + pow( abs(num3/object(2)),(2.0/object(3)));
 }

/*****************************************************************/
GraspPoses GraspEstimatorApp::computeGraspPoses(IpoptParam &pars, GraspParams &g_params, Superquadric &object_superq)
{
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

    Ipopt::SmartPtr<graspComputation> estim = new graspComputation;
    estim->init(g_params);
    estim->configure(g_params);

    clock_t tStart = clock();

    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(estim));

    double computation_time=(double)(clock() - tStart)/CLOCKS_PER_SEC;

    GraspPoses pose_hand;

    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols,", ", ", ", "", "", " [ ", "]");

    if (status==Ipopt::Solve_Succeeded)
    {
        pose_hand=estim->get_result();
        cout<<"|| Solution found            : ";
        cout<<pose_hand.getGraspParams().format(CommaInitFmt)<<endl<<endl;
        cout<<"|| Computed in               :  ";
        cout<<   computation_time<<" [s]"<<endl;
        return pose_hand;
    }
    else if(status==Ipopt::Maximum_CpuTime_Exceeded)
    {
        pose_hand=estim->get_result();
        cout<<"|| Time expired              :"<<pose_hand.getGraspParams().format(CommaInitFmt)<<endl<<endl;
        cout<<"|| Computed in               :  "<<   computation_time<<" [s]"<<endl;
        return pose_hand;
    }
    else
    {
        cerr<<"|| Not solution found"<<endl;
        Vector6d x;
        x.setZero();

        pose_hand.setGraspParams(x);
        return pose_hand;
    }


}
