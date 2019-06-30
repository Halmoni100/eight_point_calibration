#pragma once

#include <Eigen/Dense>
#include <vector>

#include "CameraIntrinsics.h"
#include "CameraExtrinsics.h"

namespace eight_point_calibrator {

  typedef std::pair<CameraIntrinsics, CameraExtrinsics> cameraParams;

  cameraParams calibrate(
    std::vector<Eigen::Vector3d> worldPoints,
    std::vector<Eigen::Vector2d> cameraPoints
  );

} // namesapce eight_point_calibrator
