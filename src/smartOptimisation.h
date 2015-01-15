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
  double find_normalisation_factor(std::vector < cv::Point2f > contour);

  // Function to normalize a contour
  std::vector< cv::Point2f > normalise_contour(std::vector < cv::Point2f > contour, double &factor);

  // Function to normalise a vector of contours
  std::vector< std::vector< cv::Point2f > > normalise_all_contours(std::vector< std::vector < cv::Point2f > > contours, std::vector< double > &factor_vector);

  // Function to denormalize a contour
  std::vector< cv::Point2f > denormalise_contour(std::vector < cv::Point2f > contour, double factor);
  
  // Function to denormalise a vector of contours
  std::vector< std::vector< cv::Point2f > > denormalise_all_contours(std::vector< std::vector < cv::Point2f > > contours, std::vector< double > factor_vector);

  // Function to estimate the radius for a contour
  int radius_estimation(std::vector< cv::Point2f > contour);

  // Function to extract a ROI from one image with border copy if the ROI is too large
  cv::Mat roi_extraction(cv::Mat original_image, cv::Rect roi);
  
  // Function to return max and min in x and y of contours
  void extract_min_max(std::vector< cv::Point2f > contour, double &min_y, double &min_x, double &max_x, double &max_y);

  // Function to define the ROI dimension around a target by a given factor
  cv::Rect roi_dimension_definition(double min_y, double min_x, double max_x, double max_y, double factor);

  // Function to convert the RGB to float gray
  cv::Mat rgb_to_float_gray(cv::Mat original_image);

  // Function which threshold the gradient image based on the magnitude image
  void gradient_thresh(cv::Mat &magnitude_image, cv::Mat &gradient_x, cv::Mat& gradient_y);

  // Function to determine the angles from the gradient images
  void orientations_from_gradient(cv::Mat gradient_x, cv::Mat gradient_y, int edges_number, cv::Mat &gradient_vp_x, cv::Mat &gradient_vp_y, cv::Mat &gradient_bar_x, cv::Mat &gradient_bar_y);

  // Function to round a matrix
  cv::Mat round_matrix(cv::Mat original_matrix);

  // Function to determin mass center by voting
  cv::Point2f mass_center_by_voting(cv::Mat magnitude_image, cv::Mat gradient_x, cv::Mat gradient_y, cv::Mat gradient_bar_x, cv::Mat gradient_bar_y, cv::Mat gradient_vp_x, cv::Mat gradient_vp_y, float radius, int edges_number);

  // Function to discover the mass center using the radial symmetry detector
  // RELATED PAPER - Fast shape-based road sign detection for a driver assistance system - xLoy et al.
  cv::Point2f radial_symmetry_detector(cv::Mat roi_image, int radius, int edges_number);
  
  // Function to discover an approximation of the mass center for each contour using a voting method for a given contour
  // THE CONTOUR NEED TO BE THE NORMALIZED CONTOUR WHICH ARE CORRECTED FOR THE DISTORTION
  cv::Point2f mass_center_discovery(cv::Mat original_image, cv::Mat translation_matrix, cv::Mat rotation_matrix, cv::Mat scaling_matrix, std::vector< cv::Point2f > contour, double factor, int type_traffic_sign);

}

#endif // SMARTOPTIMISATION_H
