/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
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
