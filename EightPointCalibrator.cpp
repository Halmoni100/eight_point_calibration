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
      Vector3f worldPoint = worldPoints[i];
      Vector2f imagePoint = imagePoints[i];
      Matrix<float,1,8> A_row;
      float x_i, y_i, X_w_i, Y_w_i, Z_w_i;
      x_i = imagePoint(0); y_i = imagePoint(1);
      X_w_i = worldPoint(0); Y_w_i = worldPoint(1); Z_w_i = worldPoint(2); 
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
    Matrix3f rot;
    Vector3f trans;
    Matrix<float,8,1> nonTrivialRightSingularVector = svdSolver.matrixV().block<8,1>(0,7);  
    Matrix<float,8,1> V = nonTrivialRightSingularVector;
    float scaleFactor = sqrt(pow(V(0),2) + pow(V(1),2) + pow(V(2),2));
    float aspectRatio_times_scaleFactor = sqrt(pow(V(4),2) + pow(V(5),2) + pow(V(6),2));
    float aspectRatio = aspectRatio_times_scaleFactor / scaleFactor;
    Matrix<float,1,8> V_norm = ((1/scaleFactor) * V).transpose(); 
    rot.block<1,3>(1,0) = V_norm.block<1,3>(0,0);
    trans(1) = V_norm(3);
    rot.block<1,3>(0,0) = (1/aspectRatio) * V_norm.block<1,3>(0,4);
    trans(0) = (1/aspectRatio) * V_norm(7);
  }

} // namespace eight_point_calibrator

