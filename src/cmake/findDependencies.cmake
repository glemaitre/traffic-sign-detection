find_package(OpenCV 2.4.8 REQUIRED core highgui imgproc features2d calib3d)


find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)

find_package(GTest REQUIRED)

set( external_includes ${EIGEN_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


set( external_libs ${OpenCV_LIBS})
