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

#include <SuperquadricLibModel/pointCloud.h>
#include <boost/range/irange.hpp>

#include <iostream>
#include <fstream>
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
    if (p.size() > 0)
    {
        if (p[0].size() == 3)
        {
            deletePoints();
            for (auto& point : p)
            {
                points.push_back(point);
                points_for_vis.push_back(point);
            }

            n_points = points.size();

            return true;
        }
        else
            return false;
    }

    return false;
}

/*********************************************/
bool PointCloud::setColors(const vector<vector<unsigned char>> &c)
{
    colors.clear();
    if (c.size() > 0)
    {
        if (c[0].size() == 3)
        {
            for (auto& color:c)
            {
                colors.push_back(color);
            }

            if (colors.size() == n_points)
              return true;
            else
              return false;

        }
        else
            return false;
    }

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
  bounding_box(0,0) = numeric_limits<double>::infinity();
  bounding_box(1,0) = numeric_limits<double>::infinity();
  bounding_box(2,0) = numeric_limits<double>::infinity();
  bounding_box(0,1) = -numeric_limits<double>::infinity();
  bounding_box(1,1) = -numeric_limits<double>::infinity();
  bounding_box(2,1) = -numeric_limits<double>::infinity();

  for (auto& point : points)
  {
      Vector3d point_tmp = orientation.transpose() * point;
      if (bounding_box(0,0) > point_tmp(0))
          bounding_box(0,0) = point_tmp(0);
      if (bounding_box(0,1) < point_tmp(0))
          bounding_box(0,1) = point_tmp(0);

      if (bounding_box(1,0) > point_tmp(1))
          bounding_box(1,0) = point_tmp(1);
      if (bounding_box(1,1) < point_tmp(1))
          bounding_box(1,1) = point_tmp(1);

      if (bounding_box(2,0) > point_tmp(2))
          bounding_box(2,0) = point_tmp(2);
      if (bounding_box(2,1) < point_tmp(2))
          bounding_box(2,1) = point_tmp(2);
  }

  return bounding_box;
}

/*********************************************/
Vector3d PointCloud::getBarycenter()
{
    getBoundingBox();

    barycenter.resize(3);
    barycenter(0) = (-bounding_box(0,0)+bounding_box(0,1))/2;
    barycenter(1) = (-bounding_box(0,0)+bounding_box(0,1))/2;
    barycenter(2) = (-bounding_box(0,0)+bounding_box(0,1))/2;

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
        M(0,0) = M(0,0) + (point(1)-barycenter(1))*(point(1)-barycenter(1)) + (point(2)-barycenter(2))*(point(2)-barycenter(2));
        M(0,1) = M(0,1) - (point(1)-barycenter(1))*(point(0)-barycenter(0));
        M(0,2) = M(0,2) - (point(2)-barycenter(2))*(point(0)-barycenter(0));
        M(1,1) = M(1,1) + (point(0)-barycenter(0))*(point(0)-barycenter(0)) + (point(2)-barycenter(2))*(point(2)-barycenter(2));
        M(2,2) = M(2,2) + (point(1)-barycenter(1))*(point(1)-barycenter(1)) + (point(0)-barycenter(0))*(point(0)-barycenter(0));
        M(1,2) = M(1,2) - (point(2)-barycenter(2))*(point(1)-barycenter(1));
    }

    M(0,0) = M(0,0)/points.size();
    M(0,1) = M(0,1)/points.size();
    M(0,2) = M(0,2)/points.size();
    M(1,1) = M(1,1)/points.size();
    M(2,2) = M(2,2)/points.size();
    M(1,2) = M(1,2)/points.size();

    M(1,0) = M(0,1);
    M(2,0) = M(0,2);
    M(2,1) = M(1,2);

    JacobiSVD<MatrixXd> svd(M, ComputeFullU | ComputeFullU);
    orientation = svd.matrixU();

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
            int count = n_points/desired_points;


            for (auto i : irange(0,n_points, count))
            {
                p_aux.push_back(points[i]);
            }
        }
        else
        {
            // Use always the same seed for reproducibility
            srand(1);
            set<unsigned int> idx;
            while (idx.size() < desired_points)
            {
                unsigned int i = (unsigned int)(rand()%n_points);
                if (idx.find(i) == idx.end())
                {
                    p_aux.push_back(points[i]);
                    idx.insert(i);
                }
             }

        }
    }

    points.clear();
    for (auto pa : p_aux)
    {
        points.push_back(pa);
    }
    p_aux.clear();

    n_points = points.size();
}

/*********************************************/
bool PointCloud::readFromFile(const char* file_name)
{
    deque<Vector3d> all_points;
    vector<vector<unsigned char>> all_colors;

    ifstream fin(file_name);
    if (!fin.is_open())
    {
        cerr << "Unable to open file \"" << file_name << "\""<<endl;

        return false;
    }

    Vector3d p(3);
    vector<unsigned int> c_(3);
    vector<unsigned char> c(3);

    string line;

    while (getline(fin,line))
    {
        istringstream iss(line);
        if (!(iss >> p(0) >> p(1) >> p(2)))
            break;
        all_points.push_back(p);

        fill(c_.begin(),c_.end(),120);
        iss >> c_[0] >> c_[1] >> c_[2];
        c[0]=(unsigned char)c_[0];
        c[1]=(unsigned char)c_[1];
        c[2]=(unsigned char)c_[2];

        if (c[0] == c[1] && c[1] == c[2])
         {
             c[0] = 50;
             c[1] = 100;
             c[2] = 0;
         }

        all_colors.push_back(c);
    }

    if (all_points.size() == 0)
    {
        cout << endl;
        cerr << "   No points found in file " << endl << endl;
        return false;
    }

    this->setPoints(all_points);
    this->setColors(all_colors);

    return true;
}

/*********************************************/
bool PointCloud::readFromFile(const string &file_name)
{
    deque<Vector3d> all_points;
    vector<vector<unsigned char>> all_colors;

    ifstream fin(file_name);
    if (!fin.is_open())
    {
        cerr << "Unable to open file \"" << file_name << "\""<<endl;

        return false;
    }

    Vector3d p(3);
    vector<unsigned int> c_(3);
    vector<unsigned char> c(3);

    string line;

    while (getline(fin,line))
    {
        istringstream iss(line);
        if (!(iss >> p(0) >> p(1) >> p(2)))
            break;
        all_points.push_back(p);

        fill(c_.begin(),c_.end(),120);
        iss >> c_[0] >> c_[1] >> c_[2];
        c[0]=(unsigned char)c_[0];
        c[1]=(unsigned char)c_[1];
        c[2]=(unsigned char)c_[2];

        if (c[0] == c[1] && c[1] == c[2])
         {
             c[0] = 50;
             c[1] = 100;
             c[2] = 0;
         }

        all_colors.push_back(c);
    }

    if (all_points.size() == 0)
    {
        cout << endl;
        cerr << "   No points found in file " << endl << endl;
        return false;
    }

    this->setPoints(all_points);
    this->setColors(all_colors);

    return true;
}
