#include "CalibratorFixture.h"
#include <iostream>

using namespace Eigen;

CalibratorFixture::CalibratorFixture()
{
	cameraIntrinsics.aspectRatio = 1.0;
	cameraIntrinsics.f_x = 1.0;
}

std::vector<Vector3f> CalibratorFixture::getCubePoints1()
{
	std::vector<Vector3f> cubePoints;
	std::vector<int> coordVals = {-1,1};
	for (int i: coordVals) { for (int j: coordVals) { for (int k: coordVals) {
		Vector3f point;
		point << i, j, k;
		cubePoints.push_back(point);
	} } }
	return cubePoints;
}

CalibratorFixture::AffineTransform CalibratorFixture::getCameraPose1()
{
  Vector3f cameraTranslation(5.0,6.0,7.0);
	Matrix3f cameraRotation;
  cameraRotation = AngleAxisf(0.25*M_PI, Vector3f::UnitZ()) 
                 * AngleAxisf(-0.75*M_PI, Vector3f::UnitY())
		             * AngleAxisf(0.5*M_PI, Vector3f::UnitZ());
	AffineTransform cameraPose1 = Translation3f(cameraTranslation) * cameraRotation; 
	return cameraPose1;	
}

std::vector<Vector3f> CalibratorFixture::transformPoints(AffineTransform transform, std::vector<Vector3f> points)
{
	std::vector<Vector3f> transformedPoints;
	std::transform(points.begin(), points.end(), std::back_inserter(transformedPoints),
			           [transform](Vector3f point) -> Vector3f { return transform * point; });
	return transformedPoints;
}

std::vector<Vector2f> CalibratorFixture::getImagePoints(std::vector<Vector3f> cameraPoints, float aspectRatio, float f_x)
{
  std::vector<Vector2f> imagePoints;
  for (const Vector3f& cameraPoint: cameraPoints) {
    float x_image = -f_x * cameraPoint(0) / cameraPoint(2);
    float f_y = f_x / aspectRatio;
    float y_image = -f_y * cameraPoint(1) / cameraPoint(2);
    Vector2f newImagePoint(x_image, y_image);
    imagePoints.push_back(newImagePoint);
  }
  return imagePoints; 
}
