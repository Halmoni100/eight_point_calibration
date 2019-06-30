#define BOOST_TEST_MODULE check_eight_point_calibrator
#include <boost/test/included/unit_test.hpp>

#include <iostream>
#include <Eigen/Dense>

#include "CalibratorFixture.h"
#include "EightPointCalibrator.h"

using namespace Eigen;

BOOST_FIXTURE_TEST_CASE( blank_image, CalibratorFixture )
{
	Vector3d cameraTranslation(5.0,5.0,5.0);
	Matrix3d cameraRotation = AxisAngled(0.75*M_PI, Vector3d::UnitZ())
		                      * AxisAngled(0.25*M_PI, Vector3d::UnitX());
}
