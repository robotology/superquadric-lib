/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#include <cmath>
#include <limits>
#include <iomanip>

#include <SuperquadricLibGrasp/graspComputation.h>

using namespace std;
using namespace Eigen;
using namespace SuperqModel;
using namespace SuperqGrasp;


/****************************************************************/
void graspComputation::init(GraspParams &g_params)
{
    // Set parameters
    n_hands = 36;
    l_o_r = g_params.left_or_right;

    obstacles.clear();

    // Get hand and object/obstacles superquadrics
    hand = g_params.hand_superq.getSuperqParams();
    object = g_params.object_superq.getSuperqParams();
    num_superq = g_params.obstacle_superqs.size();

    for (size_t i=0; i<num_superq; i++)
    {
        Superquadric obst=g_params.obstacle_superqs[i];
        obstacles.push_back(obst.getSuperqParams());
    }

    Vector3d euler_obj = g_params.object_superq.getSuperqEulerZYZ();

    // Compute Homogeneous of object w.r.t world refernece frame
    Matrix3d R;
    R = AngleAxisd(euler_obj(0), Vector3d::UnitZ())*
        AngleAxisd(euler_obj(1), Vector3d::UnitY())*
        AngleAxisd(euler_obj(2), Vector3d::UnitZ());
    H_o2w.setIdentity();
    H_o2w.block(0,0,3,3) = R;
    H_o2w.col(3).segment(0,3) = g_params.object_superq.getSuperqCenter();

    Vector3d euler_hand = g_params.hand_superq.getSuperqEulerZYZ();

    // Compute Homogeneous of hand ellipsoid w.r.t world refernece frame
    R = AngleAxisd(euler_hand(0), Vector3d::UnitZ())*
        AngleAxisd(euler_hand(1), Vector3d::UnitY())*
        AngleAxisd(euler_hand(2), Vector3d::UnitZ());
    H_h2w.setIdentity();
    H_h2w.block(0,0,3,3) = R;
    H_h2w.col(3).segment(0,3) = g_params.hand_superq.getSuperqCenter();

    points_on.clear();
    // Sampled points on the half of the hand ellipsoid closest to the robot palm
    for (int i = 0; i < (int)sqrt(n_hands); i++)
    {
        for (double theta  = 0; theta <= 2*M_PI; theta += M_PI/((int)sqrt(n_hands)))
        {
            Vector3d point = computePointsHand(hand, i, (int)sqrt(n_hands), l_o_r, theta);

            if (l_o_r == "right")
            {
                if (point(0) + point(2) <= 0)
                {
                    Vector4d point_tmp, point_tr;
                    point_tmp.segment(0,3) = point;
                    point_tmp(3) = 1;
                    point_tr = H_h2w*point_tmp;
                    point = point_tr.segment(0,3);
                    points_on.push_back(point);
                }
            }
            else
            {
                if (point(0) - point(2) < 0)
                {
                    Vector4d point_tmp, point_tr;
                    point_tmp.segment(0,3) = point;
                    point_tmp(3) = 1;
                    point_tr = H_h2w*point_tmp;
                    point = point_tr.segment(0,3);
                    points_on.push_back(point);
                }
            }
        }
    }

    // Configure cone parameters for orientation constraints
    if (l_o_r == "right")
    {
        d_x(0) = -0.8; d_x(1) = -0.7; d_x(2) = -0.5;
        d_y(0) = 0.0; d_y(1) = 0.7; d_y(2) = -0.7;
    }
    else
    {
        d_x(0) = -0.8; d_x(1) = 0.7; d_x(2) = -0.5;
        d_y(0) = 0.0; d_y(1) = -0.7; d_y(2) =-0.7;
    }

    d_x = d_x/d_x.norm();
    d_y = d_y/d_y.norm();

    d_z = d_x.cross(d_y);
    d_z = d_z/d_z.norm();

    if (num_superq > 0)
       theta_x = M_PI/4.0;
    else
        theta_x = M_PI/6.0;
    theta_y = M_PI/3.0;
    theta_z = M_PI/4.0;

    if (hand(1) > object.segment(0,3).maxCoeff() && num_superq == 0)
    {
        if (l_o_r == "right")
        {
            d_x(0) = -0.7; d_x(1) = 0.0; d_x(2) = -0.7;
            d_y(0) = 0.0; d_y(1) = 1.0; d_y(2) = 0.0;
        }
        else
        {
            d_x(0) = -0.7; d_x(1) = 0.0; d_x(2) = -0.7;
            d_y(0) = 0.0; d_y(1) = -1.0; d_y(2) = 0.0;
        }

        d_x = d_x/d_x.norm();
        d_y = d_y/d_y.norm();

        d_z = d_x.cross(d_y);
        d_z = d_z/d_z.norm();

        if (num_superq > 0)
           theta_x = M_PI/4.0;
        else
            theta_x = M_PI/6.0;
        theta_y = M_PI/4.0;
        theta_z = M_PI/6.0;
    }

    aux_objvalue = 0.0;
}

/****************************************************************/
Vector3d graspComputation::computePointsHand(Vector11d &hand, const int &j, const int &l, const string &str_hand, const double &theta)
{
    double omega;
    double ce,se,co,so;
    Vector3d point(3);

    if (object.segment(0,3).maxCoeff() > hand.segment(0,3).maxCoeff())
        hand(1) = object.segment(0,3).maxCoeff();

    omega = (j)*M_PI/(l);

    se = sin(theta);
    ce = cos(theta);
    co = cos(omega);
    so = sin(omega);

    point(0) = hand(0) * sign(ce)*(pow(abs(ce),hand(3))) * sign(co)*(pow(abs(co),hand(4)));
    point(1) = hand(1) * sign(se)*(pow(abs(se),hand(3)));
    point(2) = hand(2) * sign(ce)*(pow(abs(ce),hand(3))) * sign(so)*(pow(abs(so),hand(4)));

    return point;
}

/****************************************************************/
double graspComputation::sign(const double &v)
{
    return ((v == 0.0) ? 0.0 : ((v > 0.0) ? 1.0 : -1.0));
}

/****************************************************************/
bool graspComputation::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                                  Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style)
{
    // Get basic information for ipopt
    // Dimension of variable to optimize
    n = 6;

    // Number of constrinats
    if (num_superq == 0)
        m = 5;
    else
        m = 5 + num_superq;

    nnz_jac_g = n*m;
    nnz_h_lag = 0;
    index_style = TNLP::C_STYLE;

    return true;
}

/****************************************************************/
bool graspComputation::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                                      Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    // Get bounds
    for (Ipopt::Index i = 0; i < n; i++)
    {
       x_l[i] = bounds(i,0);
       x_u[i] = bounds(i,1);
    }

    for (Ipopt::Index i = 0; i < m; i++)
    {
       g_l[i] = bounds_constr(i,0);
       g_u[i] = bounds_constr(i,1);
    }

    return true;
}

/****************************************************************/
 bool graspComputation::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                          bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                                          Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
 {
     // Set starting pose
     for(Ipopt::Index i = 0;i < n; i++)
     {
         x[i] = hand(i+5);
     }

     return true;
 }

 /****************************************************************/
 bool graspComputation::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                              Ipopt::Number &obj_value)
 {
     // Compute cost function
     F(x,points_on,new_x);
     obj_value = aux_objvalue;

     return true;
 }

 /****************************************************************/
 double graspComputation::F(const Ipopt::Number *x, deque<Vector3d> &points_on, bool new_x)
 {
     double value = 0.0;

     for(auto point : points_on)
     {
         value += pow( pow(f(x,point),object(3))-1,2 );
     }

     value *= object(0)*object(1)*object(2)/points_on.size();

     aux_objvalue = value;
 }

 /****************************************************************/
 Matrix4d computeMatrix(const Vector6d &current_pose)
 {
     Matrix4d H_tmp;
     Matrix3d R;

     R = AngleAxisd(current_pose(3), Vector3d::UnitZ())*
         AngleAxisd(current_pose(4), Vector3d::UnitY())*
         AngleAxisd(current_pose(5), Vector3d::UnitZ());

     H_tmp.setIdentity();
     H_tmp.block(0,0,3,3) = R;
     H_tmp(0,3) = current_pose(0);
     H_tmp(1,3) = current_pose(1);
     H_tmp(2,3) = current_pose(2);

     return H_tmp;
 }

 /****************************************************************/
 double graspComputation::f(const Ipopt::Number *x, Vector3d &point)
 {
     Vector6d x_tmp;
     Vector4d point_tmp, point_tr;

     point_tmp(3) = 1;
     point_tmp.segment(0,3) = point;

     for (int i = 0; i < 6; i++)
        x_tmp(i) = x[i];

     Matrix4d H_x;
     H_x = computeMatrix(x_tmp);
     point_tr = H_x*point_tmp;

     Vector3d point3 = point_tr.segment(0,3);
     return f_v2(object,point3);
 }

 /****************************************************************/
 double graspComputation::F_v(const Vector6d &x)
 {
     // Compute cost function for finite difference gradient
     double value = 0.0;

     for(auto point : points_on)
        value += pow( pow(f_v(x,point),object(3))-1,2 );

     value *= object(0)*object(1)*object(2)/points_on.size();

     return value;
 }

/****************************************************************/
 double graspComputation::f_v(const Vector6d &x, const Vector3d &point)
 {
     Vector4d point_tmp, point_tr;

     point_tmp(3) = 1;
     point_tmp.segment(0,3) = point;

     Matrix4d H_x;
     H_x = computeMatrix(x);
     point_tr = H_x*point_tmp;

     Vector3d point3 = point_tr.segment(0,3);
     return f_v2(object,point3);
 }

 /****************************************************************/
 bool graspComputation::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number *grad_f)
 {
     Vector6d x_tmp;
     double grad_p, grad_n;
     double eps = 1e-8;

     for(Ipopt::Index i = 0;i < n; i++)
        x_tmp(i) = x[i];

     for(Ipopt::Index j = 0;j < n; j++)
     {
         x_tmp(j) += eps;
         grad_p = F_v(x_tmp);

         x_tmp(j) -= eps;
         grad_n = F_v(x_tmp);

         grad_f[j] = (grad_p-grad_n)/eps;
     }

     return true;
 }

 /****************************************************************/
 double graspComputation::coneImplicitFunction(const Vector3d &point, const Vector3d &d, double theta)
 {
     double d_dot_d = d.dot(d);
     double p_dot_p = point.dot(point);
     double p_dot_d = point.dot(d);

     return  (-p_dot_d + cos(theta));
 }

 /****************************************************************/
 bool graspComputation::eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
             Ipopt::Index m, Ipopt::Number *g)
 {
     // Compute constraints
     Vector6d x_tmp;
     for (int i = 0; i < 6; i++)
        x_tmp(i) = x[i];

     Matrix4d H_x;
     H_x = computeMatrix(x_tmp);

     Matrix4d H;
     H = H_x*H_h2w;

     Vector3d x_hand, y_hand, z_hand;
     x_hand = H.col(0).segment(0,3);
     y_hand = H.col(1).segment(0,3);
     z_hand = H.col(2).segment(0,3);

     // Constraints on orientation
     double F_x, F_y, F_z;
     F_x = coneImplicitFunction(x_hand, d_x, theta_x);
     F_y = coneImplicitFunction(y_hand, d_y, theta_y);
     F_z = coneImplicitFunction(z_hand, d_z, theta_z);

     g[0] = F_x;
     g[1] = F_y;
     g[2] = F_z;

     Vector3d x_min;
     double minz = 10.0;

     for (auto point : points_on)
     {
         Vector4d pnt;
         pnt.segment(0,3) = point;
         pnt(3) = 1;
         Vector4d p = H_x*pnt;

         if (p(2) < minz)
         {
             minz = p(2);
             x_min = p.segment(0,3);
         }
     }

     // Constraints on plane avoidance
     g[3] = (plane(0,0)*x_min(0)+plane(1,0)*x_min(1)+plane(2,0)*x_min(2)+plane(3,0))/(plane.head(3).norm());

     Vector3d robotPose;

     if (l_o_r == "right")
     {
        robotPose = x_tmp.segment(0,3)-hand(0)*z_hand;
        robotPose = robotPose-hand(0)*x_hand;
     }
     else
     {
         robotPose = x_tmp.segment(0,3)+hand(0)*z_hand;
         robotPose = robotPose-hand(0)*x_hand;
     }

     // Constraints on positon of robot plam w.r.t the object surface
     g[4] = object(0)*object(1)*object(2)*(pow(f_v2(object,robotPose), object(3)) -1);

     // Constraints on obstacle superquadric avoidance
     for (int j = 0; j < num_superq; j++)
     {
         g[5+j] = computeObstacleValues(x,j);
     }

     return true;
 }

 /****************************************************************/
 double graspComputation::G_v(const Vector6d &x, const int &i, Ipopt::Index m)
 {
     // Compute constraints for finite difference gradients (same steps as eval_g function)
     Matrix4d H_x;
     H_x = computeMatrix(x);

     Matrix4d H;
     H = H_x*H_h2w;

     Vector3d x_hand, y_hand, z_hand;
     x_hand = H.col(0).segment(0,3);
     y_hand = H.col(1).segment(0,3);
     z_hand = H.col(2).segment(0,3);

     double F_x, F_y, F_z;
     F_x = coneImplicitFunction(x_hand, d_x, theta_x);
     F_y = coneImplicitFunction(y_hand, d_y, theta_y);
     F_z = coneImplicitFunction(z_hand, d_z, theta_z);

     g[0] = F_x;
     g[1] = F_y;
     g[2] = F_z;

     Vector3d x_min;
     double minz = 10.0;

     for (auto point : points_on)
     {
         Vector4d pnt;
         pnt.segment(0,3) = point;
         pnt(3) = 1;
         Vector4d p = H_x*pnt;

         if (p(2) < minz)
         {
             minz = p(2);
             x_min = p.segment(0,3);
         }
     }

     g[3] = (plane(0,0)*x_min(0)+plane(1,0)*x_min(1)+plane(2,0)*x_min(2)+plane(3,0))/(plane.head(3).norm());

     Vector3d robotPose;

     if (l_o_r == "right")
     {
        robotPose = x.segment(0,3)-hand(0)*z_hand;
        robotPose = robotPose-hand(0)*x_hand;
     }
     else
     {
         robotPose = x.segment(0,3)+hand(0)*z_hand;
         robotPose = robotPose-hand(0)*x_hand;
     }

     g[4] = object(0)*object(1)*object(2)*(pow(f_v2(object,robotPose), object(3)) -1);

     for (int j = 0; j < num_superq; j++)
     {
         g[5+j] = computeObstacleValues_v(x,j);
     }

     return g[i];
 }

 /****************************************************************/
 bool graspComputation::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                 Ipopt::Index *jCol, Ipopt::Number *values)
 {
     double grad_p, grad_n;
     double eps = 1e-6;
     Vector6d x_tmp;

     if(values != NULL)
     {
         for(Ipopt::Index i = 0;i < n; i++)
            x_tmp(i) = x[i];

         int count = 0;
         for(Ipopt::Index i = 0;i < m; i++)
         {
             for(Ipopt::Index j = 0;j < n; j++)
             {
                 x_tmp(j) = x_tmp(j) + eps;
                 grad_p = G_v(x_tmp,i,m);

                 x_tmp(j) = x_tmp(j)-eps;
                 grad_n = G_v(x_tmp,i,m);

                 values[count] = (grad_p-grad_n)/(eps);
                 count++;
             }
         }
     }
     else
    {
        for (int j = 0; j < m; j++)
        {
            for (int i = 0; i < n; i++)
            {
                jCol[j*(n) + i] = i;
                iRow[j*(n)+ i] = j;
            }
        }
     }

     return true;
 }

 /****************************************************************/
void graspComputation::configure(GraspParams &g_params)
{
    // Set the hand of interest
    l_o_r = g_params.left_or_right;

    // Set the bounds
    if (l_o_r == "right")
        bounds = g_params.bounds_right;
    else if (l_o_r == "left")
        bounds = g_params.bounds_left;

    // Set max number of obstacle superquadrics allowed
    max_superq = g_params.max_superq;

    // Set bounds for constraints
    bounds_constr.resize(5 + max_superq -1, 2);

    if (l_o_r == "right")
        bounds_constr = g_params.bounds_constr_right;
    else if (l_o_r == "left")
        bounds_constr = g_params.bounds_constr_left;

    // Set displacemnet
    displacement = g_params.disp;
    // Set plane
    plane = g_params.pl;

    g.resize(5 + max_superq - 1);
}

/****************************************************************/
void graspComputation::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                      const Ipopt::Number *x, const Ipopt::Number *z_L,
                      const Ipopt::Number *z_U, Ipopt::Index m,
                      const Ipopt::Number *g, const Ipopt::Number *lambda,
                      Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                      Ipopt::IpoptCalculatedQuantities *ip_cq)
{
     for (int i = 0; i < 6; i++)
         solution_vector(i) = x[i];

     Matrix4d H_x;
     H_x = computeMatrix(solution_vector);

     if (notAlignedPose(H_x) == true && num_superq == 0)
         alignPose(H_x);

     Matrix3d R = H_x.block(0,0,3,3);
     solution_vector.segment(3,3) = R.eulerAngles(2,1,2);

     for (Ipopt::Index i = 0; i < 3; i++)
         solution_vector(i) = H_x(i,3);

      // Compute proper pose for robot hand on the surface of hand ellipsoid
      robot_pose = solution_vector;
      if (l_o_r == "right")
      {
          robot_pose.segment(0,3) = solution_vector.segment(0,3)-(hand(0)+displacement(2))*(H_x.col(2).segment(0,3));
          robot_pose.segment(0,3) = robot_pose.segment(0,3)-(hand(0) + displacement(0))*(H_x.col(0).segment(0,3));
          robot_pose.segment(0,3) = robot_pose.segment(0,3)-(displacement(1))*(H_x.col(1).segment(0,3));
      }
      else
      {
          robot_pose.segment(0,3) = solution_vector.segment(0,3)+(hand(0)+displacement(2))*(H_x.col(2).segment(0,3));
          robot_pose.segment(0,3) = robot_pose.segment(0,3)-(hand(0) + displacement(0))*(H_x.col(0).segment(0,3));
          robot_pose.segment(0,3) = robot_pose.segment(0,3)-(displacement(1))*(H_x.col(1).segment(0,3));
      }

      // Compute final distance between object
      final_F_value = 0.0;

      for(auto point : points_on)
      {
          final_F_value += pow( pow(f_v(solution_vector,point),object(3))-1,2 );
      }

      final_F_value /= points_on.size();

      // Compute final distance between obstacles
      final_obstacles_value = computeFinalObstacleValues(robot_pose);

      double w1 = 1.0;
      double w2 = 1e-5;
      double final_obstacles_value_average = 0.0;

      if (num_superq > 0.0)
      {
          for (auto value : final_obstacles_value)
          {
              final_obstacles_value_average += value;
          }

          final_obstacles_value_average /= final_obstacles_value.size();

      }

      if (final_obstacles_value_average < 1e-4)
         w2 = 0.0;

      solution.cost = w1*final_F_value + ((num_superq > 0) ? w2 / final_obstacles_value_average : 0.0);

      solution.setGraspParams(robot_pose);
      solution.setHandName(l_o_r);

      // Update points on hand on the final pose
      deque<Vector3d> aux;
      for (auto point : points_on)
      {
           Vector4d p;
           p(3) = 1.0;
           p.segment(0,3) = point;
           p = H_x * p;
           Vector3d p3 = p.head(3);
           aux.push_back(p3);
      }

      points_on.clear();
      points_on = aux;
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
   hand_superq.setSuperqDims(hand.segment(0,3));
   hand_superq.setSuperqExps(hand.segment(3,2));
   hand_superq.setSuperqCenter(solution_vector.segment(0,3));
   hand_superq.setSuperqOrientation(solution_vector.segment(3,3));

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
double graspComputation::computeObstacleValues(const Ipopt::Number *x, const int &k)
{
    Vector6d pose_hand;
    for (int i  = 0; i < 6; i++)
        pose_hand(i) = x[i];

    Matrix4d H_robot;
    H_robot = computeMatrix(pose_hand);

    Vector3d middle_finger;
    Vector3d thumb_finger;
    Vector3d palm_up;
    Vector3d palm_down;
    middle_finger = pose_hand.segment(0,3) + hand(0) * H_robot.col(0).segment(0,3);
    if (l_o_r == "right")
        thumb_finger = pose_hand.segment(0,3) + hand(2) * H_robot.col(2).segment(0,3);
    else
        thumb_finger = pose_hand.segment(0,3) - hand(2) * H_robot.col(2).segment(0,3);

    palm_up = pose_hand.segment(0,3) - hand(0) * H_robot.col(1).segment(0,3);
    palm_down = pose_hand.segment(0,3) + hand(0) * H_robot.col(1).segment(0,3);

    deque<Vector3d> edges_hand;
    edges_hand.push_back(pose_hand.segment(0,3));
    edges_hand.push_back(middle_finger);
    edges_hand.push_back(thumb_finger);
    edges_hand.push_back(palm_up);
    edges_hand.push_back(palm_down);

    double constr_value = 0.0;
    Vector11d obstacle = obstacles[k];

    for (auto edge : edges_hand)
    {
        constr_value +=  pow(f_v2(obstacle,edge),obstacle[3])-1;
    }

    constr_value *= obstacle(0) * obstacle(1) * obstacle(2) /edges_hand.size();

    return constr_value;
}

/****************************************************************/
double graspComputation::computeObstacleValues_v(const Vector6d &pose_hand, const int &k)
{
    Matrix4d H_robot;
    H_robot = computeMatrix(pose_hand);

    Vector3d middle_finger;
    Vector3d thumb_finger;
    Vector3d palm_up;
    Vector3d palm_down;
    middle_finger = pose_hand.segment(0,3) + hand(0) * H_robot.col(0).segment(0,3);
    if (l_o_r == "right")
        thumb_finger = pose_hand.segment(0,3) + hand(2) * H_robot.col(2).segment(0,3);
    else
        thumb_finger = pose_hand.segment(0,3) - hand(2) * H_robot.col(2).segment(0,3);

    palm_up = pose_hand.segment(0,3) - hand(0) * H_robot.col(1).segment(0,3);
    palm_down = pose_hand.segment(0,3) + hand(0) * H_robot.col(1).segment(0,3);

    deque<Vector3d> edges_hand;
    edges_hand.push_back(pose_hand.segment(0,3));
    edges_hand.push_back(middle_finger);
    edges_hand.push_back(thumb_finger);
    edges_hand.push_back(palm_up);
    edges_hand.push_back(palm_down);

    double constr_value = 0.0;
    Vector11d obstacle = obstacles[k];

    for (auto edge : edges_hand)
    {
        constr_value +=  pow(f_v2(obstacle,edge),obstacle(3))-1;
    }

    constr_value *= obstacle(0) * obstacle(1) * obstacle(2) / edges_hand.size();

    return constr_value;
}

/****************************************************************/
deque<double> graspComputation::computeFinalObstacleValues(const Vector6d &pose_hand)
{
    Matrix4d H_robot;
    H_robot = computeMatrix(pose_hand);

    Vector3d middle_finger;
    Vector3d thumb_finger;
    Vector3d palm_up;
    Vector3d palm_down;
    middle_finger = pose_hand.segment(0,3) + hand(0) * H_robot.col(0).segment(0,3);
    if (l_o_r == "right")
        thumb_finger = pose_hand.segment(0,3) + hand(2) * H_robot.col(2).segment(0,3);
    else
        thumb_finger = pose_hand.segment(0,3) - hand(2) * H_robot.col(2).segment(0,3);
    palm_up = pose_hand.segment(0,3) - hand(0) * H_robot.col(1).segment(0,3);
    palm_down = pose_hand.segment(0,3) + hand(0) * H_robot.col(1).segment(0,3);

    deque<Vector3d> edges_hand;
    edges_hand.push_back(pose_hand.segment(0,3));
    edges_hand.push_back(middle_finger);
    edges_hand.push_back(thumb_finger);
    edges_hand.push_back(palm_up);
    edges_hand.push_back(palm_down);

    deque<double> values;
    for (auto obstacle : obstacles)
    {
        double constr_value=0.0;

        for (auto edge: edges_hand)
           constr_value +=  pow(f_v2(obstacle,edge),obstacle[3])-1;

        constr_value *= obstacle(0) * obstacle(1) * obstacle(2) /points_on.size();
        values.push_back(constr_value);
    }

    return values;

}

/****************************************************************/
bool graspComputation::notAlignedPose(const Matrix4d &final_H)
{
    if ((final_H(1,1) > 0.85  && final_H(1,1) < 1.0) || (final_H(1,1) < -0.85  && final_H(1,1) > -1.0))
    {
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

    Vector3d new_z;
    Vector3d tmp = final_H.col(0).head(3);
    if (l_o_r == "right")
        new_z = tmp.cross(rot_x.col(1));
    else
    {
        rot_x.col(1) = - rot_x.col(1);
        new_z = tmp.cross(rot_x.col(1));
    }

    rot_x.col(0) = tmp;

    rot_x.col(2) = new_z/new_z.norm();

    final_H.block(0,0,3,3) = rot_x;
}

/*****************************************************************/
double graspComputation::f_v2(const Vector11d &obj, const Vector3d &point_tr)
{
    double num1 = H_o2w(0,0)*point_tr(0) + H_o2w(1,0)*point_tr(1) + H_o2w(2,0)*point_tr(2) - obj(5)*H_o2w(0,0) - obj(6)*H_o2w(1,0) - obj(7)*H_o2w(2,0);
    double num2 = H_o2w(0,1)*point_tr(0) + H_o2w(1,1)*point_tr(1) + H_o2w(2,1)*point_tr(2) - obj(5)*H_o2w(0,1) - obj(6)*H_o2w(1,1) - obj(7)*H_o2w(2,1);
    double num3 = H_o2w(0,2)*point_tr(0) + H_o2w(1,2)*point_tr(1) + H_o2w(2,2)*point_tr(2) - obj(5)*H_o2w(0,2) - obj(6)*H_o2w(1,2) - obj(7)*H_o2w(2,2);

    double tmp = pow(abs(num1/obj(0)),2.0/obj(4)) + pow(abs(num2/obj(1)),2.0/obj(4));

    return pow( abs(tmp),obj(4)/obj(3)) + pow( abs(num3/obj(2)),(2.0/obj(3)));
 }

GraspEstimatorApp::GraspEstimatorApp()
{
    pars.tol = 1e-5;
    pars.constr_tol = 1e-4;
    pars.acceptable_iter = 0;
    pars.mu_strategy = "adaptive";
    pars.max_iter = 10000;
    pars.max_cpu_time = 5.0;
    pars.nlp_scaling_method = "none";
    pars.hessian_approximation = "limited-memory";
    pars.print_level = 0;

    g_params.left_or_right = "right";
    g_params.pl << 0.0, 0.0, 1.0, 0.18;
    g_params.disp <<  0.0, 0.0, 0.0;
    g_params.max_superq = 4;
    g_params.bounds_right << -0.5, 0.0, -0.2, 0.2, -0.3, 0.3, -M_PI, M_PI,-M_PI, M_PI,-M_PI, M_PI;
    g_params.bounds_left << -0.5, 0.0, -0.2, 0.2, -0.3, 0.3,  -M_PI, M_PI,-M_PI, M_PI,-M_PI, M_PI;
    g_params.bounds_constr_left.resize(8,2);
    g_params.bounds_constr_left << -10000, 0.0, -10000, 0.0, -10000, 0.0, 0.01,
                                        10.0, 0.0, 1.0, 0.00001, 10.0, 0.00001, 10.0, 0.00001, 10.0;
    g_params.bounds_constr_right.resize(8,2);
    g_params.bounds_constr_right << -10000, 0.0, -10000, 0.0, -10000, 0.0, 0.001,
                                        10.0, 0.0, 1.0, 0.00001, 10.0, 0.00001, 10.0, 0.00001, 10.0;

    Superquadric hand;
    Vector11d hand_vector;
    hand_vector << 0.03, 0.06, 0.03, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    hand.setSuperqParams(hand_vector);
    g_params.hand_superq = hand;

}
/*****************************************************************/
GraspResults GraspEstimatorApp::computeGraspPoses(vector<Superquadric> &object_superqs)
{
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",pars.tol);
    app->Options()->SetNumericValue("constr_viol_tol",pars.constr_tol);
    app->Options()->SetIntegerValue("acceptable_iter",pars.acceptable_iter);
    app->Options()->SetStringValue("mu_strategy",pars.mu_strategy);
    app->Options()->SetIntegerValue("max_iter",pars.max_iter);
    app->Options()->SetNumericValue("max_cpu_time",pars.max_cpu_time);
    app->Options()->SetStringValue("nlp_scaling_method",pars.nlp_scaling_method);
    app->Options()->SetStringValue("hessian_approximation",pars.hessian_approximation);
    app->Options()->SetIntegerValue("print_level",pars.print_level);
    //app->Options()->SetStringValue("derivative_test","first-order");
    //app->Options()->SetStringValue("derivative_test_print_all","yes");

    GraspResults results;

    if (object_superqs.size() <= g_params.max_superq + 1)
    {
        for (size_t i = 0; i < object_superqs.size(); i++)
        {
            app->Initialize();

            g_params.object_superq = object_superqs[i];

            g_params.obstacle_superqs.clear();

            for (size_t j = 0; j < object_superqs.size() ; j++)
            {
                if (j != i)
                    g_params.obstacle_superqs.push_back(object_superqs[j]);
            }

            Ipopt::SmartPtr<graspComputation> estim = new graspComputation;
            estim->init(g_params);
            estim->configure(g_params);

            clock_t tStart = clock();

            Ipopt::ApplicationReturnStatus status = app->OptimizeTNLP(GetRawPtr(estim));

            double computation_time = (double)(clock() - tStart)/CLOCKS_PER_SEC;

            GraspPoses pose_hand;

            IOFormat CommaInitFmt(StreamPrecision, DontAlignCols,", ", ", ", "", "", " [ ", "]");

            if (status == Ipopt::Solve_Succeeded)
            {
                pose_hand = estim->get_result();
                cout << "|| ---------------------------------------------------- ||" << endl;
                cout << "|| Grasp poses for " << g_params.left_or_right << " hand estimated                 : ";
                cout << pose_hand.getGraspParams().format(CommaInitFmt) << endl << endl;
                cout << "|| Computed in                                          :  ";
                cout <<   computation_time << " [s]" << endl;
                cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;

                results.grasp_poses.push_back(pose_hand);
                results.hand_superq.push_back(estim->get_hand());
                results.points_on.push_back(estim->points_on);
                results.F_final.push_back(estim->final_F_value);
                results.F_final_obstacles.push_back(estim->final_obstacles_value);
            }
            else if(status == Ipopt::Maximum_CpuTime_Exceeded)
            {
                pose_hand = estim->get_result();
                cout << "|| ---------------------------------------------------- ||" << endl;
                cout << "|| Time expired                                         :  " << pose_hand.getGraspParams().format(CommaInitFmt) << endl << endl;
                cout << "|| Grasp poses for " << g_params.left_or_right << " hand estimated in                 :  "  <<   computation_time << " [s]" << endl;
                cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;

                results.grasp_poses.push_back(pose_hand);
                results.hand_superq.push_back(estim->get_hand());
                results.points_on.push_back(estim->points_on);
                results.F_final.push_back(estim->final_F_value);
                results.F_final_obstacles.push_back(estim->final_obstacles_value);
            }
            else
            {
                cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;
                cerr << "|| Not solution found for " << g_params.left_or_right << " hand" << endl;
                cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;
                Vector6d x;
                x.setZero();

                pose_hand.setGraspParams(x);
                results.grasp_poses.push_back(pose_hand);
                results.hand_superq.push_back(estim->get_hand());
            }
        }
    }
    else
    {
        Vector6d pose_fake;
        pose_fake << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        GraspPoses result_wrong;
        result_wrong.setGraspParams(pose_fake);
        results.grasp_poses.push_back(result_wrong);
    }

    return results;
}


/*****************************************************************/
void GraspEstimatorApp::refinePoseCost(GraspResults &grasp_res)
{
    vector<GraspPoses> &poses_computed = grasp_res.grasp_poses;

    for (size_t i = 0; i < poses_computed.size(); i++)
    {
      // Pose reachable by the robot
      VectorXd pose_hat = poses_computed[i].getGraspParamsHat();

      if (pose_hat.norm() > 0.0)
      {
          // Desired position
          Vector3d x_d = poses_computed[i].getGraspPosition();

          // Desired orientation
          Vector3d x_ea_d = poses_computed[i].getGraspEulerZYZ();
          Matrix3d R_d;
          R_d = AngleAxisd(x_ea_d(0), Vector3d::UnitZ())*
                AngleAxisd(x_ea_d(0), Vector3d::UnitZ())*
                AngleAxisd(x_ea_d(0), Vector3d::UnitZ());

          // Position reachable by the robot
          Vector3d x_hat = pose_hat.head(3);

          // Orientation reachable by the robot
          Matrix3d R_hat;

          if (pose_hat.size() == 6)
          {
              Vector3d x_ea_hat = pose_hat.tail(3);
              R_hat = AngleAxisd(x_ea_hat(0), Vector3d::UnitZ())*
                      AngleAxisd(x_ea_hat(1), Vector3d::UnitZ())*
                      AngleAxisd(x_ea_hat(2), Vector3d::UnitZ());
          }

          else if (pose_hat.size() == 7)
          {
              Vector4d axisangle = x_hat.tail(4);
              R_hat = AngleAxisd(axisangle(3), axisangle.head(3));
          }

          double error_position  = (x_d - x_hat).norm();

          R_hat.transposeInPlace();

          Matrix3d orientation_error_matrix =  R_d * R_hat;
          AngleAxisd orientation_error_vector(orientation_error_matrix);

          double error_orientation = (orientation_error_vector.axis()).norm() * fabs(sin(orientation_error_vector.angle()));

          double w1 = 1;
          double w2 = 0.01;

          if (error_position > 0.01)
          {
              w2 = 0.01;
              w1 = 10;
          }

          if (error_orientation < 0.01)
              w2 = 0.0;

          poses_computed[i].cost += w1 * error_position + w2 * error_orientation;
        }
    }

     grasp_res.best_pose = 0;
     for (size_t i = 0; i < poses_computed.size(); i++)
     {
        if (poses_computed[i].cost < poses_computed[grasp_res.best_pose].cost)
            grasp_res.best_pose = i;
     }

}

/*****************************************************************/
double GraspEstimatorApp::getPlaneHeight()
{
    return g_params.pl(3);
}

/*****************************************************************/
bool GraspEstimatorApp::setVector(const string &tag, const VectorXd &value)
{
    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols,", ", ", ", "", "", " [ ", "]");

    if (tag == "hand" && value.rows() == 11 && value.cols() == 1)
    {
        Superquadric hand;
        hand.setSuperqParams(value);
        g_params.hand_superq = hand;

        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Hand set                                             : " << g_params.hand_superq.getSuperqParams().format(CommaInitFmt) <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;

    }
    else if (tag == "plane" && value.rows() == 4 && value.cols() == 1)
    {
        g_params.pl = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Plane set                                            : " << g_params.pl.format(CommaInitFmt) <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;
    }
    else if (tag == "displacement" && value.rows() == 3 && value.cols() == 1)
    {
        g_params.disp = value;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Displacement set                                     : " << g_params.disp.format(CommaInitFmt) <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;
    }
}

/*****************************************************************/
bool GraspEstimatorApp::setMatrix(const string &tag, const MatrixXd &value)
{
    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols,", ", ", ", "", "", " [ ", "]");

    if (tag == "bounds_right" && value.rows() == 6 && value.cols() == 2)
    {
        g_params.bounds_right = value;

        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Bounds right set                                     : " << g_params.bounds_right.format(CommaInitFmt) <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;
    }
    else if (tag == "bounds_left" && value.rows() == 6 && value.cols() == 2)
    {
        g_params.bounds_left = value;

        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Bounds left set                                      : " << g_params.bounds_left.format(CommaInitFmt) <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;
    }
    else if (tag == "bounds_constr_right" && value.rows() == 8 && value.cols() == 2)
    {
        g_params.bounds_constr_right = value;

        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Bounds constraint right set                          : " << g_params.bounds_constr_right.format(CommaInitFmt) <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;
    }
    else if (tag == "bounds_constr_left" && value.rows() == 8 && value.cols() == 2)
    {
        g_params.bounds_constr_left = value;

        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Bounds constraint left set                           : " << g_params.bounds_constr_left.format(CommaInitFmt) <<endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl;
    }
}
