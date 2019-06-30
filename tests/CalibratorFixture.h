#pragma once

#include <Eigen/Dense>
#include <vector>
#include "CameraIntrinsics.h"
#include "CameraExtrinsics.h"

using namespace Eigen;

struct CalibratorFixture
{
  typedef Transform<float,3,Affine> AffineTransform;

	CalibratorFixture();
  
  CameraIntrinsics cameraIntrinsics;	

  std::vector<Vector3f> getCubePoints1();
  AffineTransform getCameraPose1();

	std::vector<Vector3f> transformPoints(AffineTransform transform, std::vector<Vector3f> points);
};
