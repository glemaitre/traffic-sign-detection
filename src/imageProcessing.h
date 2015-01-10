#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

// OpenCV library
#include <opencv2/opencv.hpp>

namespace imageprocessing {

  // Filter the binary image using morpho math and median filtering
  cv::Mat filter_image(cv::Mat seg_image);

  // Elimination of objects based on inconsistent aspects ratio and areas
  // cv::Mat
  void removal_elt(std::vector< std::vector< cv::Point > > &contours, std::vector< cv::Vec4i > hierarchy, cv::Size size_image, long int areaRatio = 1500, double lowAspectRatio = 0.5, double highAspectRatio = 1.3);

  // Compute the distance between the edge points (po and pf), with th current point pc
  float distance(cv::Point po, cv::Point pf, cv::Point pc);

  // Remove the inconsitent points inside each contour
  std::vector< std::vector< cv::Point > > contours_thresholding(std::vector< std::vector< cv::Point > > hull_contours, std::vector< std::vector< cv::Point > > contours, float dist_threshold = 2.0);

  // Function to extract the contour with some denoising step
  std::vector< std::vector< cv::Point > > contours_extraction(cv::Mat bin_image);

}

#endif // IMAGEPROCESSING_H
