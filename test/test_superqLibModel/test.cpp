#include <SuperquadricLibModel/superquadric.h>
#include <SuperquadricLibModel/pointCloud.h>
#include <SuperquadricLibGrasp/graspPoses.h>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <deque>

using namespace std;
using namespace Eigen;
using namespace SuperqModel;
using namespace SuperqGrasp;

int main()
{
    int num_params=11;
    VectorXd params(num_params);
    params(0)=0.1;
    params(1)=0.2;
    params(2)=0.3;
    Superquadric superq;

    superq.setSuperqParams(params);

    if( (superq.getSuperqParams() - params).norm() > 0.0)
    {
        cerr << "[ERROR] superquadric parameters not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    Vector3d center;
    center<<-0.1, -0.2, -0.3;

    superq.setSuperqCenter(center);

    if( (superq.getSuperqCenter() - center).norm() > 0.0)
    {
        cerr << "[ERROR] superquadric center not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    Vector3d dimensions;
    dimensions<<0.1, 0.1, 0.1;

    superq.setSuperqDims(dimensions);

    if( (superq.getSuperqDims() - dimensions).norm() > 0.0)
    {
        cerr << "[ERROR] superquadric dimensions not set correctly"<<endl;
        return EXIT_FAILURE;
    }


    Vector2d exp;
    exp<< 1.0, 1.0;

    superq.setSuperqExps(exp);

    if( (superq.getSuperqExps() - exp).norm() > 0.0)
    {
        cerr << "[ERROR] superquadric exps not set correctly"<<endl;
        return EXIT_FAILURE;
    }


    Vector3d ea;
    ea<<0.7, 0.0, 0.0;

    superq.setSuperqOrientation(ea);

    if( (superq.getSuperqEulerZYZ() - ea).norm() > 0.0)
    {
        cerr << "[ERROR] superquadric Euler angles not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    Vector4d aa;
    aa<<1.0, 0.0, 0.0, 0.7;

    superq.setSuperqOrientation(aa);

    if( (superq.getSuperqAxisAngle() - aa).norm() > 1e-10)
    {
        cerr << "[ERROR] superquadric axis angles not set correctly"<<endl;
        return EXIT_FAILURE;
    }


    deque<Vector3d> test_points;
    VectorXd point(3);
    point<< 1, 0, 0;
    test_points.push_back(point);
    point<< 2, 0, 0;
    test_points.push_back(point);
    point<< 3, 0, 0;
    test_points.push_back(point);
    point<< 1, 0, 0;
    test_points.push_back(point);
    point<< 2, 0, 0;
    test_points.push_back(point);
    point<< 3, 0, 0;
    test_points.push_back(point);

    PointCloud pc;
    pc.setPoints(test_points);

    if (fabs(pc.getNumberPoints() - test_points.size())>0)
    {
        cerr << "[ERROR] point clout not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    pc.subSample(3, false);

    if (fabs(pc.getNumberPoints() - 3)>0)
    {
        cerr << "[ERROR] not subsampled correctly"<<endl;
        return EXIT_FAILURE;
    }

    Vector3d point_test;
    point_test<< 0.0, 0.05, 0.0;

    VectorXd pose(6);
    pose<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    if (superq.insideOutsideF(pose, point_test)>=1)
    {
        cerr << "[ERROR] not inside-outside function correct 1 "<<endl;
        return EXIT_FAILURE;
    }

    point_test<< 0.0, 1.05, 0.0;

    if (superq.insideOutsideF(pose, point_test)<1)
    {
        cerr << "[ERROR] not inside-outside function correct2 "<<endl;
        return EXIT_FAILURE;
    }

    point_test<< 0.0, 0.1, 0.0;

    if (superq.insideOutsideF(pose, point_test)!=1)
    {
        cerr << "[ERROR] not inside-outside function correct 3"<<endl;
        return EXIT_FAILURE;
    }


    pose(0)=0.1;
    pose(1)=0.2;
    pose(2)=0.3;
    pose(3)=0.4;
    pose(4)=0.5;
    pose(5)=0.6;

    Vector3d p,o;

    GraspPoses grasp;

    grasp.setGraspParams(pose);

    if( (grasp.getGraspParams() - pose).norm() > 0.0)
    {
        cerr << "[ERROR] grasp parameters not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    p(0)=0.15;
    p(1)=0.25;
    p(2)=0.35;

    grasp.setGraspPosition(p);

    if( (grasp.getGraspPosition() - p).norm() > 0.0)
    {
        cerr << "[ERROR] grasp position not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    o(0)=0.45;
    o(1)=0.55;
    o(2)=0.65;

    grasp.setGraspOrientation(o);

    if( (grasp.getGraspEulerZYZ() - o).norm() > 0.0)
    {
        cerr << "[ERROR] grasp euler angles not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    aa(0)=aa(1)=aa(3)=0;
    aa(2)=1.0;

    grasp.setGraspOrientation(aa);

    if( (grasp.getGraspAxisAngle() - aa).norm() > 0.0)
    {
        cout<<"aa "<<grasp.getGraspAxisAngle()<<endl;
        cerr << "[ERROR] grasp axisangle not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    Matrix3d R;
    R = AngleAxisd(aa(3), aa.head(3));

    if( (grasp.getGraspAxes() - R).norm() > 0.0)
    {
        cerr << "[ERROR] grasp matrix not set correctly"<<endl;
        return EXIT_FAILURE;
    }

    if (!EXIT_SUCCESS)
        cout<<" == All tests passed! =="<<endl;

    // Find a way to test barycenter and orientation of matrix

    return EXIT_SUCCESS;
}
