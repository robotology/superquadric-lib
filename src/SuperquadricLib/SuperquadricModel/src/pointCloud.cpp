/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#include "pointCloud.h"
#include <boost/range/irange.hpp>

#include <iostream>
#include <set>
#include <cstdlib>

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
bool PointCloud::setPoints(const deque<Vector3d> &p)
{
    if (p[0].size()==3)
    {
        deletePoints();
        for (auto& point:p)
        {
            points.push_back(point);
            points_for_vis.push_back(point);
        }

        n_points=points.size();

        return true;
    }
    else
        return false;
}

/*********************************************/
bool PointCloud::setColors(const vector<vector<unsigned char>> &c)
{
    if (c[0].size()==3)
    {
        for (auto& color:c)
        {
            colors.push_back(color);
        }

        if (colors.size()==n_points)
          return true;
        else
          return false;

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
void PointCloud::deletePoints()
{
    points.clear();
    points_for_vis.clear();
}


/*********************************************/
MatrixXd PointCloud::getBoundingBox()
{
  bounding_box(0,0)=numeric_limits<double>::infinity();
  bounding_box(1,0)=numeric_limits<double>::infinity();
  bounding_box(2,0)=numeric_limits<double>::infinity();
  bounding_box(0,1)=-numeric_limits<double>::infinity();
  bounding_box(1,1)=-numeric_limits<double>::infinity();
  bounding_box(2,1)=-numeric_limits<double>::infinity();

  for (auto& point: points)
  {
      Vector3d point_tmp = orientation.transpose() * point;
      if (bounding_box(0,0)>point_tmp(0))
          bounding_box(0,0)=point_tmp(0);
      if (bounding_box(0,1)<point_tmp(0))
          bounding_box(0,1)=point_tmp(0);

      if (bounding_box(1,0)>point_tmp(1))
          bounding_box(1,0)=point_tmp(1);
      if (bounding_box(1,1)<point_tmp(1))
          bounding_box(1,1)=point_tmp(1);

      if (bounding_box(2,0)>point_tmp(2))
          bounding_box(2,0)=point_tmp(2);
      if (bounding_box(2,1)<point_tmp(2))
          bounding_box(2,1)=point_tmp(2);
  }

  return bounding_box;
}

/*********************************************/
Vector3d PointCloud::getBarycenter()
{
    getBoundingBox();

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
    barycenter.setZero();

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

    JacobiSVD<MatrixXd> svd(M, ComputeFullU | ComputeFullU);
    orientation=svd.matrixU();

    return orientation;
}

/*********************************************/
void PointCloud::subSample(const int &desired_points, const bool &random)
{
    deque<VectorXd> p_aux;

    if (n_points > desired_points)
    {
        if (!random)
        {
            int count=n_points/desired_points;


            for (auto i: irange(0,n_points, count))
            {
                p_aux.push_back(points[i]);
            }
        }
        else
        {
            // Use always the same seed for reproducibility
            srand(1);
            set<unsigned int> idx;
            while (idx.size()<desired_points)
            {
                unsigned int i=(unsigned int)(rand()%n_points);
                if (idx.find(i)==idx.end())
                {
                    p_aux.push_back(points[i]);
                    idx.insert(i);
                }
             }

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

/*********************************************/
/*void PointCloud::removeOutliers(double &radius=0.1, int &minpts=10)
{
    DBSCAN dbscan;
    map<size_t,set<size_t>> clusters=dbscan.cluster(all_points,options);

    size_t largest_class; size_t largest_size=0;
    for (auto it=begin(clusters); it!=end(clusters); it++)
    {
        if (it->second.size()>largest_size)
        {
            largest_size=it->second.size();
            largest_class=it->first;
        }
    }

    auto &c=clusters[largest_class];
    for (size_t i=0; i<all_points.size(); i++)
    {
        if (c.find(i)==end(c))
            out_points.push_back(all_points[i]);
        else
            in_points.push_back(all_points[i]);
    }

    cout<<out_points.size()<<"Outliers removed of "<<all_points.size()<<" points";
}*/
