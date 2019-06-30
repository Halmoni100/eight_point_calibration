#pragma once

#include <Eigen/Dense>
#include <vector>

#include "CameraIntrinsics.h"
#include "CameraExtrinsics.h"

namespace eight_point_calibrator {

  typedef std::pair<CameraIntrinsics, CameraExtrinsics> cameraParams;

  std::string to_string(cameraParams params);

  bool calibrate(
    const std::vector<Eigen::Vector3f>& worldPoints,
    const std::vector<Eigen::Vector2f>& imagePoints,
    cameraParams& paramsResult
  );

} // namespace eight_point_calibrator
