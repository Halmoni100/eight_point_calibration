cmake_minimum_required(VERSION 3.14)
project(eightPointCalibration)

set( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmakemodules )

find_package( Boost 1.70 REQUIRED COMPONENTS unit_test_framework )
find_package( Eigen3 3.3 REQUIRED )

add_library( eightPointCalibrator eightPointCalibrator.cpp )
target_include_directories( eightPointCalibrator PRIVATE
  .
  ${EIGEN3_INCLUDE_DIR}
)

add_executable( test1 tests/checkCalibrator.cpp )
target_include_directories( test1 PRIVATE
  .
  ${EIGEN3_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
)
target_link_libraries( test1
  eightPointCalibrator
  ${Boost_LIBRARIES}
)
