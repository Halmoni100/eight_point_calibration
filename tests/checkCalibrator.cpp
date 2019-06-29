#define BOOST_TEST_MODULE check_eight_point_calibrator
#include <boost/test/included/unit_test.hpp>

#include <iostream>
#include <Eigen/Dense>
#include "eightPointCalibrator.h"

struct CalibratorFixture
{
	CalibratorFixture() :
		initSize(10,10),
		imgDepth(CV_8U)
	{
	  blankImg = cv::Mat::zeros(initSize, imgDepth);
		solidImg = cv::Mat::ones(initSize, imgDepth);
	}
  std::vector<Eigen::
	cv::Size initSize; 
	int imgDepth;
  cv::Mat blankImg;
  cv::Mat solidImg;

	void testWithinTolerance(float testVal, float truthVal, float tolerance)
	{
		bool withinTolerance = abs(testVal - truthVal) < tolerance;
		BOOST_TEST(withinTolerance);
	}

	void testEigenvalues(std::pair<float,float> testVal, std::pair<float,float> truthVal)
	{
		float tolerance = 0.001;
		testWithinTolerance(testVal.first, truthVal.first, tolerance);
		testWithinTolerance(testVal.second, truthVal.second, tolerance);
  }
};

BOOST_FIXTURE_TEST_CASE( blank_image, ComputeEigenvaluesFixture )
{
  std::pair<float,float> eigenvalues = getEigenvaluesOfGradientCovariance(blankImg.clone());
  std::pair<float,float> truth_eigenvalues(0,0);
	testEigenvalues(eigenvalues, truth_eigenvalues);
}  
