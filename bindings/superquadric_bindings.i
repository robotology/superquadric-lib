%module superquadric_bindings

/* Includes the header in the wrapper code */
%{
#define SWIG_FILE_WITH_INIT
#include "SuperquadricLibModel/pointCloud.h"
#include "SuperquadricLibModel/superquadric.h"
#include "SuperquadricLibModel/options.h"
#include "SuperquadricLibModel/superquadricEstimator.h"
#include "SuperquadricLibModel/tree.h"
#include "SuperquadricLibGrasp/graspPoses.h"
#include "SuperquadricLibGrasp/graspComputation.h"
#include "SuperquadricLibVis/visRenderer.h"
#include "SuperquadricLibVis/poseVis.h"
#include "SuperquadricLibVis/planeVis.h"
#include "SuperquadricLibVis/pointsVis.h"
#include "SuperquadricLibVis/superqVis.h"
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 2>  Matrix32d;
typedef Eigen::Matrix<double, 6, 2> Matrix62d;
typedef Eigen::Matrix<double, 11, 1> Vector11d;
typedef Eigen::Matrix<double, 11, 2>  Matrix112d;
typedef Eigen::Matrix<double, 3, 2>  Matrix32d;
%}

%include <typemaps.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_deque.i>

#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW

/* --- Handle Eigen datatypes --- */

// eigen.i contains specific definitions to convert Eigen matrices into Numpy arrays.
%include "eigen.i"

// Since Eigen uses templates, we have to declare exactly which types we'd
// like to generate mappings for.
%eigen_typemaps(Eigen::Vector2d)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::Vector4d)
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix3d)
%eigen_typemaps(Eigen::Matrix4d)
%eigen_typemaps(Eigen::Matrix<double, 6, 1>)
%eigen_typemaps(Eigen::Matrix<double, 3, 2>)
%eigen_typemaps(Eigen::Matrix<double, 6, 2>)
%eigen_typemaps(Eigen::Matrix<double, 11, 1>)
%eigen_typemaps(Eigen::Matrix<double, 11, 2>)
%eigen_typemaps(Eigen::Matrix<double, 3, 2>)

/* Convert python list to std::vector */
%template(vector_uchar) std::vector<unsigned char>;
%template(vector_vector_uchar) std::vector<std::vector<unsigned char>>;
%template(vector_superquadric) std::vector<SuperqModel::Superquadric>;
%template(vector_grasp_poses) std::vector<SuperqGrasp::GraspPoses>;

%template(vector_Vector3d_Aligned) std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
%template(vector_deque_Vector3d) std::vector<std::deque<Eigen::Vector3d>>;
%template(deque_Vector3d) std::deque<Eigen::Vector3d>;
%template(deque_Vector11d) std::deque<Vector11d>;

// rename overloaded functions for python support
%rename(setGraspOrientation_ea) SuperqGrasp::GraspPoses::setGraspOrientation(Eigen::Vector3d &);
%rename(setGraspOrientation_ax) SuperqGrasp::GraspPoses::setGraspOrientation(Eigen::Vector4d &);

/* Parse the header file to generate wrappers */

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 2>  Matrix32d;
typedef Eigen::Matrix<double, 6, 2> Matrix62d;
typedef Eigen::Matrix<double, 11, 1> Vector11d;
typedef Eigen::Matrix<double, 11, 2>  Matrix112d;
typedef Eigen::Matrix<double, 3, 2>  Matrix32d;

%include "SuperquadricLibModel/pointCloud.h"
%include "SuperquadricLibModel/superquadric.h"
%include "SuperquadricLibModel/options.h"
%include "SuperquadricLibModel/superquadricEstimator.h"
%include "SuperquadricLibModel/tree.h"
%include "SuperquadricLibGrasp/graspPoses.h"
%include "SuperquadricLibGrasp/graspComputation.h"
%include "SuperquadricLibVis/visRenderer.h"
%include "SuperquadricLibVis/poseVis.h"
%include "SuperquadricLibVis/planeVis.h"
%include "SuperquadricLibVis/pointsVis.h"
%include "SuperquadricLibVis/superqVis.h"
