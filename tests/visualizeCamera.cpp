#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/viz.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>

#include "CalibratorFixture.h"
#include "CameraIntrinsics.h"

using namespace Eigen;

Vector3f getFocalPoint(CalibratorFixture::AffineTransform cameraPose,
		CameraIntrinsics intrinsics)
{
  Matrix3f rotation = cameraPose.rotation();
	Vector3f zComponentRotation = rotation.block<3,1>(0,2);
	return cameraPose.translation() + intrinsics.f_x * zComponentRotation;
}

int main(int argc, char** argv)
{
	if (argc < 2) {
		std::cout << "Missing arguments." << std::endl;
		return 1;
	}
	
	CalibratorFixture fixture;
  CalibratorFixture::AffineTransform cameraPose = fixture.getCameraPose1();

  cv::Vec3f cam_pos;
	Vector3f translation = cameraPose.translation();
 	cv::eigen2cv(translation, cam_pos);

  cv::Vec3f cam_focal_point;
 	cv::eigen2cv(getFocalPoint(cameraPose, fixture.cameraIntrinsics), cam_focal_point);	

	cv::Vec3f cam_y_dir;
  Vector3f yCompRotation = cameraPose.rotation().block<3,1>(0,1);
 	cv::eigen2cv(yCompRotation, cam_y_dir);

	cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
	cv::Affine3f transform = cv::viz::makeTransformToGlobal(cv::Vec3f(0.0f,-1.0f,0.0f), cv::Vec3f(-1.0f,0.0f,0.0f), cv::Vec3f(0.0f,0.0f,-1.0f), cam_pos);

	cv::viz::Viz3d window("Coordinate Frame");

	window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

	bool camera_pov = (argv[1][0] == 'C');

	if (!camera_pov)
	{
		cv::viz::WCameraPosition cpw(0.5);
		cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599));
		window.showWidget("CPW", cpw, cam_pose);
		window.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
	}
}
