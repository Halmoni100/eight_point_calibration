#define BOOST_TEST_MODULE check_eight_point_calibrator
#include <boost/test/included/unit_test.hpp>

#include <iostream>
#include <Eigen/Dense>

#include "CalibratorFixture.h"
#include "EightPointCalibrator.h"

using namespace Eigen;

BOOST_FIXTURE_TEST_CASE( cube1 , CalibratorFixture )
{
  AffineTransform cameraPose = getCameraPose1();
  std::vector<Vector3f> cubePoints = getCubePoints1();
  std::vector<Vector3f> cameraPoints = transformPoints(cameraPose.inverse(), cubePoints);
  std::vector<Vector2f> imagePoints = getImagePoints(cameraPoints, cameraIntrinsics.aspectRatio, cameraIntrinsics.f_x);
  eight_point_calibrator::cameraParams resultParams;
  BOOST_TEST(eight_point_calibrator::calibrate(cubePoints, imagePoints, resultParams)); 
  CameraExtrinsics truthExtrinsics = {cameraPose.rotation(), cameraPose.translation()};
  eight_point_calibrator::cameraParams truthParams(cameraIntrinsics, truthExtrinsics);
  std::cout << "Truth params:\n\n";
  std::cout << eight_point_calibrator::to_string(truthParams) << "\n\n";
  std::cout << "Result params:\n\n";
  std::cout << eight_point_calibrator::to_string(resultParams) << "\n\n";
  std::cout << std::flush;
}
