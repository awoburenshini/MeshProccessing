
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <vector>

#define SINGLE_PRECISIONs

/*Float precision declaration*/
#define RCPOVERFLOW_FLT 2.93873587705571876e-39f
#define RCPOVERFLOW_DBL 5.56268464626800345e-309

#if defined(SINGLE_PRECISION)
#define RCPOVERFLOW RCPOVERFLOW_FLT
#else
#define RCPOVERFLOW RCPOVERFLOW_DBL
#endif

/* Application precision -- can be set to single or double precision HEHEHE */
#if defined(SINGLE_PRECISION)
typedef float Real;
#else
typedef double Real;
#endif

#define Real_RANDOM() static_cast<Real>(rand()) / static_cast<Real>(RAND_MAX) // generate a [0.,1.) random Real

/* Useful Eigen typedefs based on the current precision */
typedef Eigen::Matrix<int32_t, 2, 1> Vector2i;
typedef Eigen::Matrix<int32_t, 3, 1> Vector3i;
typedef Eigen::Matrix<int32_t, 4, 1> Vector4i;
typedef Eigen::Matrix<uint32_t, 2, 1> Vector2u;
typedef Eigen::Matrix<uint32_t, 3, 1> Vector3u;
typedef Eigen::Matrix<uint32_t, 4, 1> Vector4u;
typedef Eigen::Matrix<uint8_t, 4, 1> Vector4u8;
typedef Eigen::Matrix<Real, 2, 1> Vector2r;
typedef Eigen::Matrix<Real, 3, 1> Vector3r;
typedef Eigen::Matrix<Real, 4, 1> Vector4r;
typedef Eigen::Matrix<int32_t, Eigen::Dynamic, 1> VectorXi;
typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> VectorXu;
typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, 1> VectorXu8;
typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;
typedef Eigen::Matrix<Real, Eigen::Dynamic, 1> VectorXr;
typedef Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;
typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXu;
typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXu8;
typedef Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> MatrixXr;
typedef Eigen::Matrix<Real, 2, 2> Matrix2r;
typedef Eigen::Matrix<Real, 3, 3> Matrix3r;
typedef Eigen::Matrix<Real, 4, 4> Matrix4r;

constexpr double Scale = 111319.49079327357;

#define MAPLEFTBOTTOM Scale *Vector2r(120.1812068688810058, 30.2666713059614985)
#define MAPRIGHTTOP Scale *Vector2r(120.1999946504039940, 30.2798230000000004)
