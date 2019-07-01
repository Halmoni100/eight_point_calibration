cmake_minimum_required(VERSION 3.14)
project(eightPointCalibration)

set( CMAKE_CXX_STANDARD 17 )

set( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmakemodules )

find_package( Boost 1.70 REQUIRED COMPONENTS unit_test_framework )
find_package( Eigen3 3.3 REQUIRED )
find_package( OpenCV 4.1.0 REQUIRED )

add_library( eightPointCalibrator SHARED EightPointCalibrator.cpp )
target_include_directories( eightPointCalibrator PRIVATE
  .
  ${EIGEN3_INCLUDE_DIR}
)

add_library( calibratorFixture SHARED tests/CalibratorFixture.cpp )
target_include_directories( calibratorFixture PRIVATE
	.
  tests
	${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)
target_link_libraries( calibratorFixture
	${Boost_LIBRARIES}
)

add_executable( test1 tests/checkCalibrator.cpp )
target_include_directories( test1 PRIVATE
  .
  tests
  ${EIGEN3_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
)
target_link_libraries( test1
  eightPointCalibrator
	calibratorFixture
  ${Boost_LIBRARIES}
)

add_executable( visualizeCamera tests/visualizeCamera.cpp )
target_include_directories( visualizeCamera PRIVATE
	.
	tests
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)
target_link_libraries( visualizeCamera
	calibratorFixture
	${OpenCV_LIBS}
)
