/*
* Copyright (c) 2015 
* Guillaume Lemaitre (g.lemaitre58@gmail.com)
* Yohan Fougerolle (Yohan.Fougerolle@u-bourgogne.fr)
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation; either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc., 51
* Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

#ifndef SMARTOPTIMISATION_H
#define SMARTOPTIMISATION_H

// OpenCV library
#include <opencv2/opencv.hpp>

// own library
#include "math_utils.h"

#define THRESH_GRAD_RAD_DET 0.10
#define THRESH_BINARY 0.80

static float derivative_x [] = { 0.0041,    0.0104,         0,   -0.0104,   -0.0041,
				 0.0273,    0.0689,         0,   -0.0689,   -0.0273,
				 0.0467,    0.1180,         0,   -0.1180,   -0.0467,
				 0.0273,    0.0689,         0,   -0.0689,   -0.0273,
				 0.0041,    0.0104,         0,   -0.0104,   -0.0041 };

static float derivative_y [] = { 0.0041,    0.0273,    0.0467,    0.0273,    0.0041,
				 0.0104,    0.0689,    0.1180,    0.0689,    0.0104,
				 0,         0,         0,         0,         0,
				 -0.0104,   -0.0689,   -0.1180,   -0.0689,   -0.0104,
				 -0.0041,   -0.0273,   -0.0467,   -0.0273,   -0.0041 };

namespace initoptimisation {

  // Function to find normalisation factor
  double find_normalisation_factor(const std::vector < cv::Point2f >& contour);

  // Function to normalize a contour
  void normalise_contour(const std::vector < cv::Point2f >& contour, std::vector< cv::Point2f >& output_contour, double& factor);

  // Function to normalize a contour with a given factor
  void normalise_contour_fixed_factor(const std::vector < cv::Point2f >& contour, std::vector< cv::Point2f >& output_contour, const double& factor);

  // Function to normalise a vector of contours
  void normalise_all_contours(const std::vector< std::vector < cv::Point2f > >& contours, std::vector< std::vector< cv::Point2f > >& output_contours, std::vector< double >& factor_vector);

  // Function to denormalize a contour
  void denormalise_contour(const std::vector < cv::Point2f >& contour, std::vector< cv::Point2f >& output_contour, const double& factor);
  
  // Function to denormalise a vector of contours
  void denormalise_all_contours(const std::vector< std::vector < cv::Point2f > >& contours, std::vector< std::vector< cv::Point2f > >& output_contours, const std::vector< double >& factor_vector);

  // Function to estimate the radius for a contour
  int radius_estimation(const std::vector< cv::Point2f >& contour);

  // Function to extract a ROI from one image with border copy if the ROI is too large
  void roi_extraction(const cv::Mat& original_image, const cv::Rect& roi, cv::Mat& output_image);
  
  // Function to return max and min in x and y of contours
  void extract_min_max(const std::vector< cv::Point2f >& contour, double &min_y, double &min_x, double &max_x, double &max_y);

  // Function to define the ROI dimension around a target by a given factor
  void roi_dimension_definition(const double& min_y, const double& min_x, const double& max_x, const double& max_y, const double& factor, cv::Rect& roi_dimension);

  // Function to convert the RGB to float gray
  void rgb_to_float_gray(const cv::Mat& original_image, cv::Mat& gray_image_float);

  // Function which threshold the gradient image based on the magnitude image
  void gradient_thresh(cv::Mat &magnitude_image, cv::Mat &gradient_x, cv::Mat& gradient_y);

  // Function to determine the angles from the gradient images
  void orientations_from_gradient(const cv::Mat& gradient_x, const cv::Mat& gradient_y, const int& edges_number, cv::Mat &gradient_vp_x, cv::Mat &gradient_vp_y, cv::Mat &gradient_bar_x, cv::Mat &gradient_bar_y);

  // Function to round a matrix
  cv::Mat round_matrix(const cv::Mat& original_matrix);

  // Function to determin mass center by voting
  cv::Point2f mass_center_by_voting(const cv::Mat& magnitude_image, const cv::Mat& gradient_x, const cv::Mat& gradient_y, const cv::Mat& gradient_bar_x, const cv::Mat& gradient_bar_y, const cv::Mat& gradient_vp_x, const cv::Mat& gradient_vp_y, const float& radius, const int& edges_number);

  // Function to discover the mass center using the radial symmetry detector
  // RELATED PAPER - Fast shape-based road sign detection for a driver assistance system - xLoy et al.
  cv::Point2f radial_symmetry_detector(const cv::Mat& roi_image, const int& radius, const int& edges_number);
  
  // Function to discover an approximation of the mass center for each contour using a voting method for a given contour
  // THE CONTOUR NEED TO BE THE NORMALIZED CONTOUR WHICH ARE CORRECTED FOR THE DISTORTION
  cv::Point2f mass_center_discovery(const cv::Mat& original_image, const cv::Mat& translation_matrix, const cv::Mat& rotation_matrix, const cv::Mat& scaling_matrix, const std::vector< cv::Point2f >& contour, const double& factor, const int& type_traffic_sign);

  // Function to convert a contour from euclidean to polar coordinates
  void contour_eucl_to_polar(const std::vector< cv::Point2f >& contour_eucl, std::vector< cv::PointPolar2f >& contour_polar);

  // Function to discover an approximation of the rotation offset
  double rotation_offset(const std::vector< cv::Point2f >& contour);

}

#endif // SMARTOPTIMISATION_H
