#include "pointCloud.h"
#include <boost/range/irange.hpp>

using namespace std;
using namespace boost;
using namespace Eigen;
using namespace SuperqModel;

/*********************************************/
PointCloud::PointCloud()
{
    n_points=0;
}

/*********************************************/
PointCloud::~PointCloud()
{
    points.clear();
    n_points=0;
}

/*********************************************/
bool PointCloud::setPoints(deque<VectorXd> &p)
{
    if (p[0].size()==3 || p[0].size()==6)
    {
        for (auto& point:p)
        {
            points.push_back(point);
        }

        n_points=points.size();

        return true;
    }
    else
        return false;
}

/*********************************************/
int PointCloud::getNumberPoints()
{
    return n_points;
}

/*********************************************/
Vector3d PointCloud::getBarycenter()
{
    // They are in the point cloud reference frame, remember
    bounding_box(3,2);
    bounding_box(0,0)=numeric_limits<double>::infinity();
    bounding_box(1,0)=numeric_limits<double>::infinity();
    bounding_box(2,0)=numeric_limits<double>::infinity();
    bounding_box(0,1)=-numeric_limits<double>::infinity();
    bounding_box(1,1)=-numeric_limits<double>::infinity();
    bounding_box(2,1)=-numeric_limits<double>::infinity();

    for (auto& point: points)
    {
        if (bounding_box(0,0)>point(0))
            bounding_box(0,0)=point(0);
        if (bounding_box(0,1)<point(0))
            bounding_box(0,1)=point(0);

        if (bounding_box(1,0)>point(1))
            bounding_box(1,0)=point(1);
        if (bounding_box(1,1)<point(1))
            bounding_box(1,1)=point(1);

        if (bounding_box(2,0)>point(2))
            bounding_box(2,0)=point(2);
        if (bounding_box(2,1)<point(2))
            bounding_box(2,1)=point(2);
    }

    barycenter.resize(3);
    barycenter(0)=(-bounding_box(0,0)+bounding_box(0,1))/2;
    barycenter(1)=(-bounding_box(0,0)+bounding_box(0,1))/2;
    barycenter(2)=(-bounding_box(0,0)+bounding_box(0,1))/2;

    return barycenter;
}

/*********************************************/
Matrix3d PointCloud::getAxes()
{
    Matrix3d M;
    M.setZero();


    for (auto& point: points)
    {
        M(0,0)= M(0,0) + (point(1)-barycenter(1))*(point(1)-barycenter(1)) + (point(2)-barycenter(2))*(point(2)-barycenter(2));
        M(0,1)= M(0,1) - (point(1)-barycenter(1))*(point(0)-barycenter(0));
        M(0,2)= M(0,2) - (point(2)-barycenter(2))*(point(0)-barycenter(0));
        M(1,1)= M(1,1) + (point(0)-barycenter(0))*(point(0)-barycenter(0)) + (point(2)-barycenter(2))*(point(2)-barycenter(2));
        M(2,2)= M(2,2) + (point(1)-barycenter(1))*(point(1)-barycenter(1)) + (point(0)-barycenter(0))*(point(0)-barycenter(0));
        M(1,2)= M(1,2) - (point(2)-barycenter(2))*(point(1)-barycenter(1));
    }

    M(0,0)= M(0,0)/points.size();
    M(0,1)= M(0,1)/points.size();
    M(0,2)= M(0,2)/points.size();
    M(1,1)= M(1,1)/points.size();
    M(2,2)= M(2,2)/points.size();
    M(1,2)= M(1,2)/points.size();

    M(1,0)= M(0,1);
    M(2,0)= M(0,2);
    M(2,1)= M(1,2);

    JacobiSVD<MatrixXd> svd( M, ComputeThinU | ComputeThinV);
    orientation=svd.matrixU();

    return orientation;
}


/*********************************************/
void PointCloud::subSample(int desired_points)
{
    deque<VectorXd> p_aux;

    if (n_points > desired_points)
    {
        int count=n_points/desired_points;


        for (auto i: irange(0,n_points, count))
        {
            p_aux.push_back(points[i]);
        }
    }

    points.clear();
    for (auto pa: p_aux)
    {
        points.push_back(pa);
    }
    p_aux.clear();

    n_points=points.size();
}

