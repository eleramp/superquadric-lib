%module superquadric_bindings

/* Includes the header in the wrapper code */
%{
#define SWIG_FILE_WITH_INIT
#include "SuperquadricLibModel/pointCloud.h"
#include "SuperquadricLibModel/superquadric.h"
#include "SuperquadricLibModel/options.h"
#include <Eigen/Core>
typedef Eigen::Matrix<double, 3, 2>  Matrix32d;
typedef Eigen::Matrix<double, 6, 2> Matrix62d;
typedef Eigen::Matrix<double, 11, 1> Vector11d;
typedef Eigen::Matrix<double, 11, 2>  Matrix112d;
typedef Eigen::Matrix<double, 3, 2>  Matrix32d;
#include "SuperquadricLibModel/superquadricEstimator.h"
#include "SuperquadricLibModel/tree.h"
%}

%include <typemaps.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_deque.i>

/* Convert python list to std::vector */
%template(vector_uchar) std::vector<unsigned char>;
%template(vector_vector_uchar) std::vector<std::vector<unsigned char>>;
%template(vector_superquadric) std::vector<SuperqModel::Superquadric>;

/* --- Handle Eigen datatypes --- */

// eigen.i contains specific definitions to convert Eigen matrices into Numpy arrays.
%include <eigen.i>

%eigen_typemaps(Eigen::Vector2d)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::Vector4d)
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix3d)
//%eigen_typemaps(Eigen::Matrix<double, 3, 2>)
//%eigen_typemaps(Eigen::Matrix<double, 6, 2>)
//%eigen_typemaps(Eigen::Matrix<double, 11, 1>)
//%eigen_typemaps(Eigen::aligned_allocator<Eigen::Vector3d>)

// %template(Matrix32d) Eigen::Matrix<double, 3, 2>;
// %template(Matrix62d) Eigen::Matrix<double, 6, 2>;
// %template(Vector11d) Eigen::Matrix<double, 11, 1>;
%template(vector_Vector3d_Aligned) std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
%template(deque_Vector3d) std::deque<Eigen::Vector3d>;


/* Parse the header file to generate wrappers */

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
