#include <Eigen/Geometry>
#include <Eigen/Jacobi>

#include "EightPointCalibrator.h"

using namespace Eigen;

namespace eight_point_calibrator {

  std::string to_string(cameraParams params)
  {
    std::stringstream ss;
    ss << "f_x: " << params.first.f_x << ", aspectRatio: " << params.first.aspectRatio << "\n";
    AngleAxisf rotation(params.second.Rot);
    ss << "Rotation:\n";
    ss << "\taxis: [" << rotation.axis()(0) << "," << rotation.axis()(1) << "," << rotation.axis()(2) << "]\n";
    ss << "\tangle: " << rotation.angle() << "\n";
    Vector3f translation = params.second.Trans;
    ss << "Translation: [" <<  translation(0) << "," << translation(1) << "," << translation(2) << "\n";
    return ss.str();
  }

  bool calibrate(std::vector<Vector3f> worldPoints,
                 std::vector<Vector2f> imagePoints,
                 cameraParams& paramsResult)
  {
    // Make sure we have all correspondences for world and image points
    assert(worldPoints.size() == imagePoints.size());
    int numPoints = worldPoints.size();
    // Also make sure A matrix will have at least rank 7
    // This only checks number of points, not collinearity
    assert(numPoints >= 7);

    // Construct A matrix
    Matrix<float,Dynamic,8> A = MatrixXf::Zero(numPoints,8); 

    for (int i = 0; i < numPoints; i++) {
      float x_i, y_i, X_w_i, Y_w_i, Z_w_i;
      x_i = imagePoints[i](0); y_i = imagePoints[i](1);
      X_w_i = worldPoints[i](0); Y_w_i = worldPoints[i](1); Z_w_i = worldPoints[i](2); 
      Matrix<float,1,8> A_row;
      A_row << x_i * X_w_i, x_i * Y_w_i, x_i * Z_w_i, x_i, 
              -y_i * X_w_i,-y_i * Y_w_i,-y_i * Z_w_i, -y_i;
      A.block<1,8>(i,0) = A_row;
    }

    // Solve SVD of A
    JacobiSVD svdSolver(A, ComputeFullV);

    // Make sure A has at least rank 7
    // TODO: remove hardcoded threshold
    svdSolver.setThreshold(0.01);
    if ( svdSolver.rank() < 7 )
      return false;
     
    // Recover calibration parameters from SVD of A
    Matrix<float,8,1> nonTrivialRightSingularVector = svdSolver.matrixV().block<8,1>(0,7);  
    Matrix<float,8,1> V = nonTrivialRightSingularVector;

    float scaleFactor = sqrt(pow(V(0),2) + pow(V(1),2) + pow(V(2),2));
    float aspectRatio_times_scaleFactor = sqrt(pow(V(4),2) + pow(V(5),2) + pow(V(6),2));
    float aspectRatio = aspectRatio_times_scaleFactor / scaleFactor;

    Vector3f trans;

    Matrix<float,1,8> V_norm = ((1/scaleFactor) * V).transpose(); 
    Matrix<float,1,3> rot_row_2 = V_norm.block<1,3>(0,0);
    trans(1) = V_norm(3);
    Matrix<float,1,3> rot_row_1 = (1/aspectRatio) * V_norm.block<1,3>(0,4);
    trans(0) = (1/aspectRatio) * V_norm(7);

    Matrix<float,1,3> rot_row_3 = rot_row_1.cross(rot_row_2);    

    Matrix3f rotInit;
    rotInit.block<1,3>(0,0) = rot_row_1;
    rotInit.block<1,3>(1,0) = rot_row_2;
    rotInit.block<1,3>(2,0) = rot_row_3;
    JacobiSVD rotSvdSolver(rotInit, ComputeFullU | ComputeFullV);
    Matrix3f rot = rotSvdSolver.matrixU() * Matrix3f::Identity() * rotSvdSolver.matrixV().transpose();

    bool samplePointFound = false;
    int samplePointIndex = 0;
    while (samplePointIndex < numPoints)
    {
      const Vector2f& imagePoint = imagePoints[samplePointIndex];
      float threshold = 0.01;
      if (abs(imagePoint(0)) > threshold)
      {
        samplePointFound = true;
        break;
      }
      samplePointIndex++;
    }
    if (!samplePointFound) return false;
    const Vector2f& sampleImagePoint = imagePoints[samplePointIndex];
    const Vector3f& sampleWorldPoint = worldPoints[samplePointIndex]; 

    float x_img, r11, r12, r13, X_w, Y_w, Z_w, T_x;
    x_img = sampleImagePoint(0);
    X_w = sampleWorldPoint(0); Y_w = sampleWorldPoint(1); Z_w = sampleWorldPoint(2);
    r11 = rot(0,0); r12 = rot(0,1); r13 = rot(0,2);
    T_x = trans(0);
    if ( x_img * (r11*X_w + r12*Y_w + r13*Z_w + T_x) > 0 ) 
    {
      // reverse signs of first two rows of rotation
      rot.block<1,3>(0,0) = -rot.block<1,3>(0,0);
      rot.block<1,3>(1,0) = -rot.block<1,3>(1,0);
      // reverse sign of first two components of translation
      trans(0) = -trans(0);
      trans(1) = -trans(1);
    }

    // Solve for T_z and f_x
    Matrix<float,Dynamic,2> A2;
    Matrix<float,Dynamic,1> b;
    for (int i = 0; i < numPoints; i++)
    {
      float x_i, r11, r12, r13, r31, r32, r33, X_w_i, Y_w_i, Z_w_i, T_x;
      x_i = imagePoints[i](0);
      r11 = rot(0,0); r12 = rot(0,1); r13 = rot(0,2);
      r31 = rot(2,0); r32 = rot(2,1); r33 = rot(2,2);
      X_w_i = worldPoints[i](0); Y_w_i = worldPoints[i](1); Z_w_i = worldPoints[i](2);
      T_x = trans(0);
      A2(i,0) = x_i;
      A2(i,1) = r11*X_w_i + r12*Y_w_i + r13*Z_w_i + T_x;
      b(i) = -x_i * (r31*X_w_i + r32*Y_w_i + r33*Z_w_i); 
    }
    JacobiSVD leastSquaresSolver(A2);
    Matrix2f leastSquaresSolution = leastSquaresSolver.solve(b);
    trans(2) = leastSquaresSolution(0);
    float f_x = leastSquaresSolution(1);

    CameraIntrinsics intrinsics = {aspectRatio, f_x};
    CameraExtrinsics extrinsics = {rot, trans};
    paramsResult = cameraParams(intrinsics, extrinsics);

    return true;
  }

} // namespace eight_point_calibrator

