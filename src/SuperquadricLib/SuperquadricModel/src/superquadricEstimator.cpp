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

#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>

#include <SuperquadricLibModel/superquadricEstimator.h>

using namespace std;
using namespace Eigen;
using namespace SuperqModel;

/****************************************************************/
void SuperqEstimator::init()
{
    points_downsampled.deletePoints();
    aux_objvalue = 0.0;

    return;
}

/****************************************************************/
void SuperqEstimator::setPoints(PointCloud &point_cloud, const int &optimizer_points, const bool &random)
{
    // Set points from point cloud and downsample them if too many
    if (point_cloud.getNumberPoints() > optimizer_points)
        point_cloud.subSample(optimizer_points, random);

    points_downsampled = point_cloud;

    used_points = points_downsampled.getNumberPoints();

    cout << "|| Downsampled points used for modeling                 :  " << used_points << endl;

    x0.resize(11);
    x0.setZero();
    // Compute initial estimate of superquadric
    computeX0(x0, point_cloud);

    return;
}

/****************************************************************/
void SuperqEstimator::computeX0(Vector11d &x0, PointCloud &point_cloud)
{
    // Exponents are computed according to object class, defined in configure
    x0(3) = (bounds(3,0) + bounds(3,1))/2;
    x0(4) = (bounds(4,0) + bounds(4,1))/2;
    x0(5) = x0(6) = x0(7) = 0.0;

    // Initial orientation is obtained from point cloud
    Matrix3d orientation;
    orientation = point_cloud.getAxes();

    x0.segment(8,3) = orientation.eulerAngles(2,1,2);

    // Initial value for superquadric dimensions is obtained from point cloud
    // bounding boxes
    Matrix32d bounding_box(3,2);
    bounding_box=point_cloud.getBoundingBoxSuperqFrame(orientation);

    x0(0) = (-bounding_box(0,0) + bounding_box(0,1))/2;
    x0(1) = (-bounding_box(1,0) + bounding_box(1,1))/2;
    x0(2) = (-bounding_box(2,0) + bounding_box(2,1))/2;

    // Intial value of the superquadric center is obtained from the point cloud
    Vector3d barycenter = point_cloud.getBarycenter();
    x0(5) = barycenter(0);
    x0(6) = barycenter(1);
    x0(7) = barycenter(2);

    return;
}

/****************************************************************/
bool SuperqEstimator::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                  Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style)
{
    // Number of variable to estimate is 11, the parameters of the object superquadric
    n = 11;
    m = nnz_jac_g = nnz_h_lag = 0;
    index_style = TNLP::C_STYLE;

    return true;
}

/****************************************************************/
void SuperqEstimator::computeBounds()
{
    // Compute bounds starting from x0
    bounds(0,1) = x0(0)*1.5;
    bounds(1,1) = x0(1)*1.5;
    bounds(2,1) = x0(2)*1.5;

    bounds(0,0) = x0(0)*0.7;
    bounds(1,0) = x0(1)*0.7;
    bounds(2,0) = x0(2)*0.7;

    bounds(5,0) = x0(5)-bounds(0,1);
    bounds(6,0) = x0(6)-bounds(1,1);
    bounds(7,0) = x0(7)-bounds(2,1);
    bounds(5,1) = x0(5)+bounds(0,1);
    bounds(6,1) = x0(6)+bounds(1,1);
    bounds(7,1) = x0(7)+bounds(2,1);

    bounds(8,0)  = 0;
    bounds(9,0)  = 0;
    bounds(10,0) = 0;
    bounds(8,1)  = 2*M_PI;
    bounds(9,1)  = M_PI;
    bounds(10,1) = 2*M_PI;

    return;
}

/****************************************************************/
bool SuperqEstimator::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                     Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    computeBounds();

    for (Ipopt::Index i = 0; i < n; i++)
    {
       x_l[i] = bounds(i,0);
       x_u[i] = bounds(i,1);
    }

    return true;
}

/****************************************************************/
bool SuperqEstimator::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
{
     // Set initial x
     for (Ipopt::Index i = 0; i < n;  i++)
     {
         x[i] = x0[i];
     }

     return true;
}

/****************************************************************/
bool SuperqEstimator::eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
             Ipopt::Number &obj_value)
{
    // Compute cost function
    F(x, new_x);
    obj_value = aux_objvalue;

    return true;
}

/****************************************************************/
void SuperqEstimator::F(const Ipopt::Number *x, bool &new_x)
{
    if (new_x)
    {
         double value = 0.0;
         for(auto point : points_downsampled.points)
         {
             double tmp = pow(f(x,point),x[3]) - 1;
             value += tmp*tmp;
         }
         value *= x[0]*x[1]*x[2]/used_points;
         aux_objvalue = value;
    }

    return;
}

/****************************************************************/
double SuperqEstimator::f(const Ipopt::Number *x, const Vector3d &point_cloud)
{
     Vector3d euler;
     euler(0) = x[8];
     euler(1) = x[9];
     euler(2) = x[10];

     Matrix3d R;
     R = AngleAxisd(euler(0), Vector3d::UnitZ())*
         AngleAxisd(euler(1), Vector3d::UnitY())*
         AngleAxisd(euler(2), Vector3d::UnitZ());

     double num1 = R(0,0)*point_cloud(0) + R(1,0)*point_cloud(1) + R(2,0)*point_cloud(2) - x[5]*R(0,0) - x[6]*R(1,0) - x[7]*R(2,0);
     double num2 = R(0,1)*point_cloud(0) + R(1,1)*point_cloud(1) + R(2,1)*point_cloud(2) - x[5]*R(0,1) - x[6]*R(1,1) - x[7]*R(2,1);
     double num3 = R(0,2)*point_cloud(0) + R(1,2)*point_cloud(1) + R(2,2)*point_cloud(2) - x[5]*R(0,2) - x[6]*R(1,2) - x[7]*R(2,2);
     double tmp = pow(abs(num1/x[0]),2.0/x[4]) + pow(abs(num2/x[1]),2.0/x[4]);

     return pow( abs(tmp),x[4]/x[3]) + pow( abs(num3/x[2]),(2.0/x[3]));
}

/****************************************************************/
double SuperqEstimator::F_v(const Vector11d &x)
{
     // Evaluate cost function for finite difference gradient computation
     double value=0.0;

     for(auto point : points_downsampled.points)
     {
          double tmp = pow(f_v(x,point),x(3)) - 1;
          value += tmp*tmp;
     }

     value *= x(0)*x(1)*x(2)/used_points;

     return value;
}

/****************************************************************/
double SuperqEstimator::f_v(const Vector11d &x, const Vector3d &point_cloud)
{
    Matrix3d R;
    R = AngleAxisd(x(8), Vector3d::UnitZ())*
      AngleAxisd(x(9), Vector3d::UnitY())*
      AngleAxisd(x(10), Vector3d::UnitZ());

    double num1 = R(0,0)*point_cloud(0) + R(1,0)*point_cloud(1) + R(2,0)*point_cloud(2) - x(5)*R(0,0) - x(6)*R(1,0) - x(7)*R(2,0);
    double num2 = R(0,1)*point_cloud(0) + R(1,1)*point_cloud(1) + R(2,1)*point_cloud(2) - x(5)*R(0,1) - x(6)*R(1,1) - x(7)*R(2,1);
    double num3 = R(0,2)*point_cloud(0) + R(1,2)*point_cloud(1) + R(2,2)*point_cloud(2) - x(5)*R(0,2) - x(6)*R(1,2) - x(7)*R(2,2);
    double tmp = pow(abs(num1/x(0)),2.0/x(4)) + pow(abs(num2/x(1)),2.0/x(4));

    return pow( abs(tmp),x(4)/x(3)) + pow( abs(num3/x(2)),(2.0/x(3)));
}

/****************************************************************/
bool SuperqEstimator::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
              Ipopt::Number *grad_f)
{
    // Evaluate gradient with finite differences
    Vector11d x_tmp(n);
    double grad_p, grad_n;
    double eps = 1e-8;

    for (Ipopt::Index j = 0; j < n; j++)
        x_tmp(j) = x[j];

    for (Ipopt::Index j = 0; j < n; j++)
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
void SuperqEstimator::configure(const string &object_class)
{
    // Configure exponents according to object class
    bounds.resize(11,2);

    obj_class = object_class;

    if (obj_class == "default")
    {
      bounds(3,0) = 0.1;
      bounds(3,1) = 1.9;
      bounds(4,0) = 0.1;
      bounds(4,1) = 1.9;
    }
    else if (obj_class =="box")
    {
      bounds(3,0) = 0.05;
      bounds(3,1) = 0.6;
      bounds(4,0) = 0.05;
      bounds(4,1) = 0.6;
    }
    else if (obj_class == "cylinder")
    {
      bounds(3,0) = 0.01;
      bounds(3,1) = 0.3;
      bounds(4,0) = 0.8;
      bounds(4,1) = 1.2;
    }
    else if (obj_class == "sphere")
    {
      bounds(3,0) = 1.;
      bounds(3,1) = 1.;
      bounds(4,0) = 1.;
      bounds(4,1) = 1.;
    }

    return;
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
    for (Ipopt::Index i = 0; i < n; i++)
        params_sol[i] = x[i];

    solution.setSuperqParams(params_sol);

    return;
}

/****************************************************************/
Superquadric SuperqEstimator::get_result() const
{
    return solution;
}

SuperqEstimatorApp::SuperqEstimatorApp()
{
    pars.tol = 1e-5;
    pars.acceptable_iter = 0;
    pars.max_iter = 1000000;
    pars.max_cpu_time = 5.0;
    pars.mu_strategy = "adaptive";
    pars.nlp_scaling_method = "gradient-based";
    pars.hessian_approximation = "limited-memory";
    pars.print_level = 0;
    pars.object_class = "default";
    pars.optimizer_points = 50;
    pars.random_sampling = true;

    m_pars.merge_model = true;
    m_pars.minimum_points = 150;
    m_pars.fraction_pc = 8;
    m_pars.threshold_axis = 0.7;
    m_pars.threshold_section1 = 0.6;
    m_pars.threshold_section2 = 0.03;
    m_pars.debug = false;

}
/****************************************************************/
vector<Superquadric> SuperqEstimatorApp::computeSuperq(PointCloud &point_cloud)
{
    // Process for estimate the superquadric
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = new Ipopt::IpoptApplication;
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

    cout << "|| ---------------------------------------------------- ||" << endl;
    estim->setPoints(point_cloud, pars.optimizer_points, pars.random_sampling);

    clock_t tStart = clock();

    Ipopt::ApplicationReturnStatus status = app->OptimizeTNLP(GetRawPtr(estim));

    double computation_time = (double)(clock() - tStart)/CLOCKS_PER_SEC;

    Superquadric superq;
    vector<Superquadric> superqs;

    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols,", ", ", ", "", "", " [ ", "]");

    if (status == Ipopt::Solve_Succeeded)
    {
        superq = estim->get_result();
        cout << "|| Superquadric estimated                               : ";
        cout << superq.getSuperqParams().format(CommaInitFmt) << endl;
        cout << "|| Computed in                                          :  ";
        cout <<  computation_time << " [s]" << endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;
        superqs.push_back(superq);
        return superqs;
    }
    else if(status == Ipopt::Maximum_CpuTime_Exceeded)
    {
        superq = estim->get_result();
        cout << "|| Time expired                                         :" << superq.getSuperqParams().format(CommaInitFmt) << endl;
        cout << "|| Superquadric estimated in                            :  " <<   computation_time << " [s]" << endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;
        superqs.push_back(superq);
        return superqs;
    }
    else
    {
        cerr << "|| Not solution found" << endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;
        Vector11d x(11);
        x.setZero();

        superq.setSuperqParams(x);
        superqs.push_back(superq);
        return superqs;
    }

}

/****************************************************************/
vector<Superquadric> SuperqEstimatorApp::computeMultipleSuperq(PointCloud &point_cloud)
{
    superq_tree = new SuperqTree;
    superq_tree_new = new SuperqTree;

    point_cloud_split1 = new PointCloud;
    point_cloud_split2 = new PointCloud;

    clock_t tStart = clock();

    iterativeModeling(point_cloud);

    double computation_time1 = (double)(clock() - tStart) / CLOCKS_PER_SEC;

    double computation_time2;

    cout << "|| ---------------------------------------------------- ||" << endl;
    cout << "|| Multiple superquadrics estimated in                  : ";
    cout <<    computation_time1 << " [s]" << endl;
    cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;

    if (m_pars.merge_model)
    {
        tStart = clock();

        findImportantPlanes(superq_tree->root);

        generateFinalTree(superq_tree->root, superq_tree_new->root);

        superq_tree->root = superq_tree_new->root;

        computation_time2 = (double)(clock() - tStart) / CLOCKS_PER_SEC;

        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Merged model estimated in                            : ";
        cout <<    computation_time2 << " [s]" << endl;
        cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;
    }

    cout << "|| ---------------------------------------------------- ||" << endl;
    cout << "|| Complete modeling process took                       : ";
    cout <<    computation_time1 + computation_time2 << " [s]" << endl;
    cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;

    delete point_cloud_split1;
    delete point_cloud_split2;

    return fillSolution(superq_tree->root);
}

/***********************************************************************/
void SuperqEstimatorApp::iterativeModeling(PointCloud &point_cloud)
{
    if(point_cloud.getNumberPoints()/2 < m_pars.minimum_points)
        m_pars.minimum_points = point_cloud.getNumberPoints()/2;

    if (point_cloud.getNumberPoints() / m_pars.fraction_pc >= m_pars.minimum_points)
        h_tree = (int)log2(m_pars.fraction_pc);
    else
    {
        while(point_cloud.getNumberPoints() / m_pars.fraction_pc < m_pars.minimum_points)
        {
            m_pars.fraction_pc--;
        }

        cout << "m_pars.fraction_pc " << m_pars.fraction_pc << endl;
        h_tree = (int)log2(m_pars.fraction_pc);
    }

    cout << "|| ---------------------------------------------------- ||" << endl;
    cout << "|| Number of point cloud splittings                     : " << h_tree << endl;
    cout << "|| Number of points for each point cloud                : " << point_cloud.getNumberPoints()/m_pars.fraction_pc << " " << endl;
    cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;

    superq_tree->setPoints(point_cloud);

    computeNestedSuperq(superq_tree->root);

    return;
}

/***********************************************************************/
void SuperqEstimatorApp::computeNestedSuperq(node *newnode)
{
    if ((newnode != NULL))
    {
        vector<Superquadric> superqs1;
        vector<Superquadric> superqs2;

        nodeContent node_c1;
        nodeContent node_c2;

        if ((newnode->height <= h_tree))
        {
            splitPoints(newnode);

            cout << endl << "|| ---------------------------------------------------- ||" << endl;
            cout << "|| Right node with height                               :  " << newnode->height << endl;
            superqs1 = computeSuperq(*point_cloud_split1);
            cout << endl << "|| ---------------------------------------------------- ||" << endl;
            cout << "|| Left node with height                                :  " << newnode->height << endl;
            superqs2 = computeSuperq(*point_cloud_split2);;

            node_c1.superq = superqs1[0];
            node_c2.superq = superqs2[0];
            node_c1.point_cloud = new PointCloud;
            node_c2.point_cloud = new PointCloud;

            *node_c1.point_cloud = *point_cloud_split1;
            *node_c2.point_cloud = *point_cloud_split2;

            node_c1.height = newnode->height + 1;
            node_c2.height = newnode->height + 1;

            superq_tree->insert(node_c1, node_c2, newnode);
        }

        computeNestedSuperq(newnode->left);
        computeNestedSuperq(newnode->right);
    }

    return;
}

/***********************************************************************/
void SuperqEstimatorApp::splitPoints(node *leaf)
{
    point_cloud_split1->deletePoints();
    point_cloud_split2->deletePoints();
    Vector3d center;
    center.setZero();

    for (auto point : leaf->point_cloud->points_for_vis)
    {
        center(0) += point(0);
        center(1) += point(1);
        center(2) += point(2);
    }

    center(0) /= leaf->point_cloud->points_for_vis.size();
    center(1) /= leaf->point_cloud->points_for_vis.size();
    center(2) /= leaf->point_cloud->points_for_vis.size();

    Matrix3d M;
    M.setZero();

    for (auto point : leaf->point_cloud->points_for_vis)
    {
        M(0,0) = M(0,0) + (point(1)-center(1))*(point(1)-center(1)) + (point(2)-center(2))*(point(2)-center(2));
        M(0,1) = M(0,1) - (point(1)-center(1))*(point(0)-center(0));
        M(0,2) = M(0,2) - (point(2)-center(2))*(point(0)-center(0));
        M(1,1) = M(1,1) + (point(0)-center(0))*(point(0)-center(0)) + (point(2)-center(2))*(point(2)-center(2));
        M(2,2) = M(2,2) + (point(1)-center(1))*(point(1)-center(1)) + (point(0)-center(0))*(point(0)-center(0));
        M(1,2) = M(1,2) - (point(2)-center(2))*(point(1)-center(1));
    }

    M(0,0) = M(0,0)/leaf->point_cloud->points_for_vis.size();
    M(0,1) = M(0,1)/leaf->point_cloud->points_for_vis.size();
    M(0,2) = M(0,2)/leaf->point_cloud->points_for_vis.size();
    M(1,1) = M(1,1)/leaf->point_cloud->points_for_vis.size();
    M(2,2) = M(2,2)/leaf->point_cloud->points_for_vis.size();
    M(1,2) = M(1,2)/leaf->point_cloud->points_for_vis.size();

    M(1,0) = M(0,1);
    M(2,0) = M(0,2);
    M(2,1) = M(1,2);

    JacobiSVD<MatrixXd> svd(M, ComputeFullU | ComputeFullU);
    Matrix3d orientation = svd.matrixU();
    Vector4d plane;
    plane.head(3) = orientation.col(2);

    plane(3) = (plane(0)*center(0)+plane(1)*center(1)+plane(2)*center(2));

    leaf->plane = plane;

    deque<Vector3d> deque_points1, deque_points2;

    for (auto point : leaf->point_cloud->points_for_vis)
    {
        if (plane(0)*point(0)+plane(1)*point(1)+plane(2)*point(2) - plane(3) > 0)
        {
            deque_points1.push_back(point);
          }
        else
        {
            deque_points2.push_back(point);
          }
    }

    point_cloud_split1->setPoints(deque_points1);
    point_cloud_split2->setPoints(deque_points2);

    cout << "|| ---------------------------------------------------- ||" << endl;
    cout << "|| Number of points in point cloud right                :  " << point_cloud_split1->getNumberPoints() << endl;
    cout << "|| Number of points in point cloud left                 :  " << point_cloud_split2->getNumberPoints() << endl;
    cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;

    return;
}

/****************************************************************/
void SuperqEstimatorApp::computeSuperqAxis(node *node)
{
    node->axis_x = node->superq.getSuperqAxes().col(0);    //TEST ROW INSTEAD
    node->axis_y = node->superq.getSuperqAxes().col(1);
    node->axis_z = node->superq.getSuperqAxes().col(2);

    return;
}

/****************************************************************/
bool SuperqEstimatorApp::axisParallel(node *node1, node *node2, Matrix3d &relations)
{
    double threshold = m_pars.threshold_axis;
    if (abs(node1->axis_x.dot(node2->axis_x)) > threshold)
    {
        relations(0,0) = 1;
    }
    if  (abs(node1->axis_x.dot(node2->axis_y)) > threshold)
    {
        relations(0,1) = 1;
    }
    if  (abs(node1->axis_x.dot(node2->axis_z)) > threshold)
    {
        relations(0,2) = 1;
    }
    if (abs(node1->axis_y.dot(node2->axis_x)) > threshold)
    {
        relations(1,0) = 1;
    }
    if  (abs(node1->axis_y.dot(node2->axis_y)) > threshold)
    {
        relations(1,1) = 1;
    }
    if  (abs(node1->axis_y.dot(node2->axis_z))> threshold)
    {
        relations(1,2) = 1;
    }
    if (abs(node1->axis_z.dot(node2->axis_x)) > threshold)
    {
        relations(2,0) = 1;
    }
    if  (abs(node1->axis_z.dot(node2->axis_y)) > threshold)
    {
        relations(2,1) = 1;
    }
    if  (abs(node1->axis_z.dot(node2->axis_z)) > threshold)
    {
        relations(2,2) = 1;
    }

    if (relations.row(0).norm() > 1.0)
    {
        int i_max = 0;
        double max = -1.0;
        if(abs(node1->axis_x.dot(node2->axis_x)) < abs(node1->axis_x.dot(node2->axis_y)))
        {
            max = abs(node1->axis_x.dot(node2->axis_y));
            i_max = 1;
        }
        else
            max = abs(node1->axis_x.dot(node2->axis_x));

         if (max < abs(node1->axis_x.dot(node2->axis_z)))
         {
            max = abs(node1->axis_x.dot(node2->axis_z));
            i_max = 2;
         }

         relations(0,0) = relations(0,1) = relations(0,2) = 0.0;
         relations(0,i_max) = 1.0;
    }

    if (relations.row(1).norm() > 1.0)
    {
        int i_max = 0;
        double max = -1.0;
        if(abs(node1->axis_y.dot(node2->axis_x)) < abs(node1->axis_y.dot(node2->axis_y)))
        {
            max = abs(node1->axis_y.dot(node2->axis_y));
            i_max = 1;
        }
        else
            max = abs(node1->axis_y.dot(node2->axis_x));

         if (max < abs(node1->axis_y.dot(node2->axis_z)))
         {
            max = abs(node1->axis_y.dot(node2->axis_z));
            i_max = 2;
         }

         relations(1,0) = relations(1,1) = relations(1,2) = 0.0;
         relations(1,i_max) = 1.0;
    }

    if (relations.row(2).norm() > 1.0)
    {
        int i_max = 0;
        double max = -1.0;
        if(abs(node1->axis_z.dot(node2->axis_x)) < abs(node1->axis_z.dot(node2->axis_y)))
        {
            max = abs(node1->axis_z.dot(node2->axis_y));
            i_max = 1;
        }
        else
            max = abs(node1->axis_z.dot(node2->axis_x));

         if (max < abs(node1->axis_z.dot(node2->axis_z)))
         {
            max = abs(node1->axis_z.dot(node2->axis_z));
            i_max = 2;
         }

         relations(2,0) = relations(2,1) = relations(2,2) = 0.0;
         relations(2,i_max) = 1.0;
    }

    if ((relations.row(0).norm() > 0.0) || (relations.row(1).norm() > 0.0) || (relations.row(2).norm() > 0.0))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/****************************************************************/
bool SuperqEstimatorApp::sectionEqual(node *node1, node *node2, Matrix3d &relations)
{
    double threshold1 = m_pars.threshold_section1;

    double threshold2 = m_pars.threshold_section2;

    Matrix3d R1;
    R1.row(0) = node1->axis_x;
    R1.row(1) = node1->axis_x;
    R1.row(2) = node1->axis_x;

    R1.transposeInPlace();

    Matrix3d R2;
    R2.row(0) = node2->axis_x;
    R2.row(1) = node2->axis_x;
    R2.row(2) = node2->axis_x;

    Matrix3d R2_rot;
    R2_rot = relations*R2;

    R2_rot.transposeInPlace();

    Vector3d dim1 = node1->superq.getSuperqDims();
    Vector3d dim2 = node2->superq.getSuperqDims();

    Vector3d dim2_rot = relations*dim2;
    Vector3d p1,p2,p3,p4;
    p1.setZero();
    p2.setZero();
    p3.setZero();
    p4.setZero();

    deque<bool> equals;
    equals.clear();

     for (size_t i = 0; i < 3; i++)
    {
        bool equal;
        int other_index;

        if (relations.row(i).norm() > 0.0)
        {
            other_index = i;

            p1 = node1->superq.getSuperqCenter()+dim1(i)*R1.col(i);
            p2 = node1->superq.getSuperqCenter()-dim1(i)*R1.col(i);


            p3 = node2->superq.getSuperqCenter()+dim2_rot(other_index)*R2_rot.col(i);
            p4 = node2->superq.getSuperqCenter()-dim2_rot(other_index)*R2_rot.col(i);

            double cos_max_dist1, cos_max_dist2;
            cos_max_dist1 = ((p1 - p2) / (p1 - p2).norm()).dot((p1 - p4) / (p1 - p4).norm()) ;
            cos_max_dist2 = ((p3 - p4) / (p3 - p4).norm()).dot((p1 - p4) / (p1 - p4).norm());

            if (abs(max(cos_max_dist1, cos_max_dist2)) > threshold1)
            {
                equal = true;

                for (size_t j=0; j<3; j++)
                {
                    if ( i != j && dim2_rot(j) != 0.0)
                    {
                        if (abs(dim1(j) - dim2_rot(j)) < threshold2)
                        {
                            equal = equal && true;
                        }
                        else
                            equal = equal && false;
                    }
                }
            }
            else
            {
                if ( dim2_rot(i) != 0.0)
                {
                    if (abs(dim1(i) - dim2_rot(i)) < threshold2)
                    {
                        equal = true;
                    }
                    else
                        equal = false;
                }
            }
            equals.push_back(equal);
        }
        else
            equals.push_back(true);

    }

    return equals[0] && equals[1] && equals[2];
}

/****************************************************************/
double SuperqEstimatorApp::edgesClose(node *node1, node *node2)
{
    deque<Vector3d> edges_1;
    deque<Vector3d> edges_2;

    computeEdges(node1, edges_1);
    computeEdges(node2, edges_2);

    double distance_min =1000.0;

    for (auto edge1 : edges_1)
    {
       for (auto edge2 : edges_2)
       {
           double distance = (edge1-edge2).norm();

           if (distance < distance_min)
           {
               distance_min = distance;
           }
       }
    }

    return distance_min;

}

/****************************************************************/
void SuperqEstimatorApp::computeEdges(node *node, deque<Vector3d> &edges)
{
    edges.clear();

    Vector3d point;

    point = node->superq.getSuperqCenter() + node->superq.getSuperqDims()(0) * node->axis_x;
    edges.push_back(point);

    point = node->superq.getSuperqCenter() - node->superq.getSuperqDims()(0) * node->axis_x;
    edges.push_back(point);

    point = node->superq.getSuperqCenter() + node->superq.getSuperqDims()(1) * node->axis_y;
    edges.push_back(point);

    point = node->superq.getSuperqCenter() - node->superq.getSuperqDims()(1) * node->axis_y;
    edges.push_back(point);

    point = node->superq.getSuperqCenter() + node->superq.getSuperqDims()(2) * node->axis_z;
    edges.push_back(point);

    point = node->superq.getSuperqCenter() - node->superq.getSuperqDims()(2) * node->axis_z;
    edges.push_back(point);

    return;
}

/***********************************************************************/
void SuperqEstimatorApp::copySuperqChildren(node *old_node, node *newnode)
{
    nodeContent node_c1;
    nodeContent node_c2;

    node_c1.superq = old_node->left->superq;
    node_c2.superq = old_node->right->superq;

    node_c1.point_cloud = new PointCloud;
    node_c2.point_cloud = new PointCloud;

    node_c1.point_cloud = old_node->left->point_cloud;
    node_c2.point_cloud = old_node->right->point_cloud;

    node_c1.height = newnode->height + 1;
    node_c2.height = newnode->height + 1;

    node_c1.plane_important = old_node->left->plane_important;
    node_c2.plane_important = old_node->right->plane_important;

    superq_tree_new->insert(node_c2, node_c1, newnode);

    return;
}

/****************************************************************/
bool SuperqEstimatorApp::findImportantPlanes(node *current_node)
{
    Matrix3d relations;

    superq_tree->root->plane_important=false;
    if (current_node->height < h_tree)
    {
        if (m_pars.debug)
        {
            cout << "|| ---------------------------------------------------- ||" << endl;
            cout << "|| Look for relevant planes in left sub-tree         " << endl;
        }

        if (current_node->left != NULL)
            findImportantPlanes(current_node->left);

        if (m_pars.debug)
        {
            cout << "|| ---------------------------------------------------- ||" << endl;

            cout << "|| ---------------------------------------------------- ||" << endl;
            cout << "|| Look for relevant planes in right sub-tree         " << endl;
        }

        if (current_node->right != NULL)
            findImportantPlanes(current_node->right);

        if (m_pars.debug)
            cout << "|| ---------------------------------------------------- ||" << endl;
    }

    if (current_node->height > 1 && current_node->plane_important == false)
    {
        computeSuperqAxis(current_node->left);
        computeSuperqAxis(current_node->right);

        if (m_pars.debug)
            cout << "|| Node height         " << current_node->height << endl;

        if (axisParallel(current_node->left, current_node->right, relations) && sectionEqual(current_node->left, current_node->right, relations))
        {
            if (m_pars.debug)
                cout << "|| Superquadric to be merged!         " << endl;

            current_node->plane_important = false;

            if (superq_tree->searchPlaneImportant(current_node->left) == false && superq_tree->searchPlaneImportant(current_node->right) == false)
            {
                current_node->left = NULL;
                current_node->right = NULL;
            }
        }
        else
        {
            if (m_pars.debug)
                cout << "|| Plane current node is important!       " << endl;
            current_node->plane_important = true;

            node *node_uncle = ((current_node == current_node->father->right) ? current_node->father->left : current_node->father->right);

            if (node_uncle != NULL)
            {
                computeSuperqAxis(node_uncle);

                double distance_right = edgesClose(current_node->right, node_uncle);
                double distance_left = edgesClose(current_node->left, node_uncle);

                if(distance_right < distance_left)
                    current_node->right->uncle_close = node_uncle;
                else
                    current_node->left->uncle_close = node_uncle;

                bool parallel_to_uncle;

                if (current_node->left->uncle_close != NULL)
                {
                    parallel_to_uncle = (axisParallel(current_node->left, node_uncle, relations) && sectionEqual(current_node->left, node_uncle, relations));
                    if (m_pars.debug)
                        cout << "|| Left node is parallel and with similar dimensions w.r.t its uncle      " << endl;

                }
                else if (current_node->right->uncle_close != NULL)
                {
                    parallel_to_uncle = (axisParallel(current_node->right, node_uncle, relations) && sectionEqual(current_node->right, node_uncle, relations));
                    if (m_pars.debug)
                        cout << "|| Right node is parallel and with similar dimensions w.r.t its uncle      " << endl;
                }

                if(parallel_to_uncle == false)
                {
                    if (m_pars.debug)
                        cout << "|| Plane father is important      " << endl;
                    current_node->father->plane_important = true;
                }
                else
                {
                    if (m_pars.debug)
                        cout << "|| Plane father is not important      " << endl;
                    current_node->father->plane_important = false;
                }
            }
        }
    }

    if (current_node->height == 1)
    {
        computeSuperqAxis(current_node->left);
        computeSuperqAxis(current_node->right);

        if (current_node->plane_important && m_pars.debug)
            cout << "|| Plane of root is important!     " << endl;
        else if (m_pars.debug)
            cout << "|| Plane of root is not important!     " << endl;

        if ( axisParallel(current_node->left, current_node->right, relations) && sectionEqual(current_node->left, current_node->right, relations) == false)
        {
            current_node->plane_important = true;
        }

        if ((superq_tree->searchPlaneImportant(current_node->left) == false
                && superq_tree->searchPlaneImportant(current_node->right) == false))
            current_node->plane_important = true;
    }

    return true;
}

/***********************************************************************/
bool SuperqEstimatorApp::generateFinalTree(node *old_node, node *newnode)
{
    if (old_node != NULL && old_node->height <= h_tree)
    {
        if (m_pars.debug)
        {
            cout << "|| ---------------------------------------------------- ||" << endl;
            cout << "|| Node height                        :" << old_node->height << endl;
        }

        if (old_node->height > 1)
        {
            if (old_node->plane_important == true && old_node->father->plane_important == false)
            {
                if (m_pars.debug)
                    cout << "|| Current plane important!     " << endl;
                superqUsingPlane(old_node, old_node->father->point_cloud, newnode);

                generateFinalTree(old_node->left, newnode->left);
                generateFinalTree(old_node->right, newnode->right);

                if (m_pars.debug)
                    cout << "|| ---------------------------------------------------- ||" << endl;

            }
            else if (old_node->plane_important == true && old_node->father->plane_important == true)
            {
                if (m_pars.debug)
                    cout << "|| Current  and father's plane important!     " << endl;

                copySuperqChildren(old_node, newnode);

                generateFinalTree(old_node->left, newnode->left);
                generateFinalTree(old_node->right, newnode->right);

                if (m_pars.debug)
                    cout << "|| ---------------------------------------------------- ||" << endl;
            }
            else if (old_node->left != NULL && old_node->right != NULL)
            {
                if (superq_tree->searchPlaneImportant(old_node->left) == true && superq_tree->searchPlaneImportant(old_node->right) == true)
                {
                    copySuperqChildren(old_node, newnode);

                    generateFinalTree(old_node->left, newnode->left);
                    generateFinalTree(old_node->right, newnode->right);

                    if (m_pars.debug)
                        cout << "|| ---------------------------------------------------- ||" << endl;
                }
                else
                {
                    if (superq_tree->searchPlaneImportant(old_node->left))
                    {
                        if (m_pars.debug)
                            cout << "|| Only left sub-tree important!    " << endl;
                        generateFinalTree(old_node->left, newnode);
                    }
                    else if (superq_tree->searchPlaneImportant(old_node->right))
                    {
                        if (m_pars.debug)
                            cout << "|| Only right sub-tree important!    " << endl;
                        generateFinalTree(old_node->right, newnode);
                    }

                    if (m_pars.debug)
                        cout << "|| ---------------------------------------------------- ||" << endl;
                }
            }
        }
        else
        {
            if (old_node->plane_important == true)
            {
                copySuperqChildren(old_node, newnode);
                generateFinalTree(old_node->left, newnode->left);
                generateFinalTree(old_node->right, newnode->right);

                if (m_pars.debug)
                    cout << "|| ---------------------------------------------------- ||" << endl;
            }
            else if (superq_tree->searchPlaneImportant(old_node->left) == true && superq_tree->searchPlaneImportant(old_node->right) == true)
            {
                copySuperqChildren(old_node, newnode);

                generateFinalTree(old_node->left, newnode->left);
                generateFinalTree(old_node->right, newnode->right);

                if (m_pars.debug)
                    cout << "|| ---------------------------------------------------- ||" << endl;
            }
            else
            {
                if (superq_tree->searchPlaneImportant(old_node->left) == true)
                {
                    if (m_pars.debug)
                        cout << "|| Only left sub-tree important!    " << endl;
                    generateFinalTree(old_node->left, newnode);
                }
                if (superq_tree->searchPlaneImportant(old_node->right) == true)
                {
                    if (m_pars.debug)
                        cout << "|| Only right sub-tree important!    " << endl;
                    generateFinalTree(old_node->right, newnode);
                }

                if (m_pars.debug)
                    cout << "|| ---------------------------------------------------- ||" << endl;
            }
        }
    }

    return true;
}

/****************************************************************/
void SuperqEstimatorApp::superqUsingPlane(node *old_node, PointCloud *points, node *newnode)
{
    deque<Vector3d> deque_points1, deque_points2;
    for (auto point : points->points_for_vis)
    {
        if (old_node->plane(0)*point(0) + old_node->plane(1)*point(1) + old_node->plane(2)*point(2) - old_node->plane(3) > 0)
            deque_points1.push_back(point);
        else
            deque_points2.push_back(point);
    }

    point_cloud_split1->setPoints(deque_points1);
    point_cloud_split2->setPoints(deque_points2);

    vector<Superquadric> superqs1, superqs2;
    superqs1 = computeSuperq(*point_cloud_split1);
    superqs2 = computeSuperq(*point_cloud_split2);

    nodeContent node_c1;
    nodeContent node_c2;
    node_c1.superq = superqs1[0];
    node_c2.superq = superqs2[0];
    node_c1.point_cloud = new PointCloud;
    node_c2.point_cloud = new PointCloud;

    *node_c1.point_cloud = *point_cloud_split1;
    *node_c2.point_cloud = *point_cloud_split2;
    node_c1.height = newnode->height + 1;
    node_c2.height = newnode->height + 1;

    superq_tree_new->insert(node_c1, node_c2, newnode);

    return;
}

/****************************************************************/
vector<Superquadric> SuperqEstimatorApp::fillSolution(node *leaf)
{
    vector<Superquadric> superqs;
    addSuperqs(leaf, superqs);

    delete superq_tree;

    return superqs;
}

/**********************************************************************/
void SuperqEstimatorApp::addSuperqs(node *leaf, vector<Superquadric> &superqs)
{
    Superquadric sup;

    if (leaf != NULL)
    {
        if (leaf->superq.getSuperqDims().norm() > 0.0)
        {
            sup = leaf->superq;

            if (leaf->right != NULL)
            {
                addSuperqs(leaf->right,  superqs);
            }

            if (leaf->left != NULL)
            {
                addSuperqs(leaf->left, superqs);
            }

            if (leaf->right == NULL && leaf->left == NULL && leaf->height > 0)
            {
                superqs.push_back(sup);
            }
        }
        else
        {
            if (leaf->right != NULL)
            {
                addSuperqs(leaf->right,  superqs);
            }
            if (leaf->left != NULL)
            {
                addSuperqs(leaf->left,  superqs);
            }
        }
    }

    return;
}
