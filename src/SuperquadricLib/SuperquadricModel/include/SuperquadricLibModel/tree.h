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

#ifndef SUPERQTREE_H
#define SUPERQTREE_H

#include <Eigen/Dense>

#include <SuperquadricLibModel/superquadric.h>
#include <SuperquadricLibModel/pointCloud.h>

namespace SuperqModel {

struct node
{
    int height;
    node *left;
    node *right;
    node *father;
    node *uncle_close;

    SuperqModel::PointCloud *point_cloud;
    SuperqModel::Superquadric superq;
    Eigen::Vector4d plane;
    Eigen::Vector3d axis_x;
    Eigen::Vector3d axis_y;
    Eigen::Vector3d axis_z;
    Eigen::Matrix3d R;
    bool plane_important;
};

struct nodeContent
{
    SuperqModel::PointCloud *point_cloud;
    SuperqModel::Superquadric superq;
    Eigen::Vector4d plane;
    int height;
    bool plane_important;
};

class SuperqTree
{
    /***********************************************************************/
    void destroy_tree(node *leaf);

public:

    node *root;

    /***********************************************************************/
    SuperqTree();

    /***********************************************************************/
    void setPoints(SuperqModel::PointCloud &point_cloud);

    /***********************************************************************/
    ~SuperqTree();

    /***********************************************************************/
    void destroy_tree();

    /***********************************************************************/
    void printNode(node *leaf);

    /***********************************************************************/
    void printTree();

    /***********************************************************************/
    void insert(const nodeContent &node_content1, const nodeContent &node_content2, node *leaf);

    /***********************************************************************/
    void insert_uncle(node *leaf, node *uncle);

    /***********************************************************************/
    void reset();

    /***********************************************************************/
    bool searchPlaneImportant(const node *leaf);
};

}
#endif
