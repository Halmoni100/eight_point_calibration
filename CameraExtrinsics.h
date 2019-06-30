#pragma once

#include <Eigen/Dense>

struct CameraExtrinsics
{
  Eigen::Matrix3f Rot;
  Eigen::Vector3f Trans;
};
