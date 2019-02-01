/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#include <iostream>

#include "tree.h"

using namespace std;
using namespace Eigen;
using namespace SuperqModel;

/***********************************************************************/
SuperqTree::SuperqTree()
{
    root = new node;
    Vector11d init_par;
    init_par.setZero();
    root->superq.setSuperqParams(init_par);
    root->plane.setZero();
    root->left = NULL;
    root->right = NULL;
    root->height = 1;
    root->axis_x.setZero();
    root->axis_y.setZero();
    root->axis_z.setZero();
    root->R.setIdentity();
    root->plane_important = false;
    root->uncle_close = NULL;
}

/***********************************************************************/
void SuperqTree::reset()
{
    root = new node;
    Vector11d init_par;
    init_par.setZero();
    root->superq.setSuperqParams(init_par);
    root->plane.setZero();
    root->left = NULL;
    root->right = NULL;
    root->height = 1;
    root->axis_x.setZero();
    root->axis_y.setZero();
    root->axis_z.setZero();
    root->R.setIdentity();
    root->plane_important = false;
    root->uncle_close = NULL;
}

/***********************************************************************/
SuperqTree::~SuperqTree()
{
    destroy_tree();
}

/***********************************************************************/
void SuperqTree::destroy_tree()
{
    destroy_tree(root);
}

/***********************************************************************/
void SuperqTree::destroy_tree(node *leaf)
{
    if (leaf != NULL)
    {
        destroy_tree(leaf->left);
        destroy_tree(leaf->right);
        delete leaf;
    }
}


/***********************************************************************/
void SuperqTree::setPoints(SuperqModel::PointCloud &point_cloud)
{
    root->point_cloud = &point_cloud;
}

/***********************************************************************/
void SuperqTree::insert_uncle(node *leaf,  node *uncle)
{
    leaf->uncle_close = uncle;
}

/***********************************************************************/
void SuperqTree::insert(const nodeContent &node_content1, const nodeContent &node_content2,  node *leaf)
{
    if (leaf->right == NULL)
        leaf->right = new node;

    leaf->right->superq = node_content1.superq;
    leaf->right->plane = node_content1.plane;

    leaf->right->point_cloud = node_content1.point_cloud;
    leaf->right->left = NULL;
    leaf->right->right = NULL;
    leaf->right->father = leaf;
    leaf->right->height = node_content1.height;
    leaf->right->plane_important = false;

    if (leaf->left == NULL)
        leaf->left = new node;

    leaf->left->superq = node_content2.superq;
    leaf->left->plane = node_content2.plane;
    leaf->left->point_cloud = node_content2.point_cloud;
    leaf->left->left = NULL;
    leaf->left->right = NULL;
    leaf->left->father = leaf;
    leaf->left->height = node_content2.height;
    leaf->left->plane_important = false;
}

/***********************************************************************/
bool SuperqTree::searchPlaneImportant(const node *leaf)
{
    double one_false = false;
    if (leaf != NULL)
    {
        searchPlaneImportant(leaf->left);
        searchPlaneImportant(leaf->right);

        if (leaf->plane_important == true)
            one_false = one_false || leaf->plane_important;
    }

    return one_false;
}

/***********************************************************************/
void SuperqTree::printNode(node *leaf)
{
    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols,", ", ", ", "", "", " [ ", "]");

    if(leaf != NULL)
    {
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Node content                                        :||" << endl;
        cout << "|| Superquadric                                        :||" << endl;
        cout << leaf->superq.getSuperqParams().format(CommaInitFmt) << endl << endl << endl;
        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Left ...                                             ||" << endl;
        if (leaf->left != NULL)
            printNode(leaf->left);

        cout << "|| ---------------------------------------------------- ||" << endl;
        cout << "|| Right ...                                            ||" << endl;
        if (leaf->right != NULL)
            printNode(leaf->right);
    }
    else
        cout << "|| ---------------------------------------------------- ||" << endl << endl << endl;
}

/***********************************************************************/
void SuperqTree::printTree()
{
    cout << "|| ---------------------------------------------------- ||" << endl;
    cout << "|| Tree content                                        :||" << endl;
    cout << "|| ---------------------------------------------------- ||" << endl;

    printNode(root);

}
