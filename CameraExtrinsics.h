#pragma once

#include <Eigen/Dense>

struct CameraExtrinsics
{
  Eigen::Matrix3d Rot;
  Eigen::Vector3d Trans;
};
