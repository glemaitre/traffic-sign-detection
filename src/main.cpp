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

// stl library
#include <string>
#include <iostream>
#include <chrono>
#include <ctime>

// OpenCV library
#include <opencv2/opencv.hpp>

// our own code
#include "segmentation.h"
#include "colorConversion.h"
#include "imageProcessing.h"
#include "smartOptimisation.h"
#include "math_utils.h"

int main(int argc, char *argv[]) {

  // Chec the number of arguments
  if (argc != 2) {
    std::cout << "********************************" << std::endl;
    std::cout << "Usage of the code: ./traffic-sign-detection imageFileName.extension" << std::endl;
    std::cout << "********************************" << std::endl;

    return -1;
  }

  // Clock for measuring the elapsed time
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  // Read the input image - convert char* to string
  std::string input_filename(argv[1]);

  // Read the input image
  cv::Mat input_image = cv::imread(input_filename);

  // Check that the image has been opened 
  if (!input_image.data) {
    std::cout << "Error to read the image. Check ''cv::imread'' function of OpenCV" << std::endl;
    return -1;
  }
  // Check that the image read is a 3 channels image
  CV_Assert(input_image.channels() == 3);


  /*
   * Conversion of the image in some specific color space
   */

  // Conversion of the rgb image in ihls color space
  cv::Mat ihls_image;
  colorconversion::convert_rgb_to_ihls(input_image, ihls_image);
  // Conversion from RGB to logarithmic chromatic red and blue
  std::vector< cv::Mat > log_image;
  colorconversion::rgb_to_log_rb(input_image, log_image);

  /*
   * Segmentation of the image using the previous transformation
   */

  // Segmentation of the IHLS and more precisely of the normalised hue channel 
  // ONE PARAMETER TO CONSIDER - COLOR OF THE TRAFFIC SIGN TO DETECT - RED VS BLUE
  int nhs_mode = 0; // nhs_mode == 0 -> red segmentation / nhs_mode == 1 -> blue segmentation
  cv::Mat nhs_image_seg;
  segmentation::seg_norm_hue(ihls_image, nhs_image_seg, nhs_mode);
  // Segmentation of the log chromatic image
  cv::Mat log_image_seg;
  segmentation::seg_log_chromatic(log_image, log_image_seg);

  /*
   * Merging and filtering of the previous segmentation
   */

  // Merge the results of previous segmentation using an OR operator
  // Pre-allocation of an image by cloning a previous image
  cv::Mat merge_image_seg = nhs_image_seg.clone();
  cv::bitwise_or(nhs_image_seg, log_image_seg, merge_image_seg);
  // Filter the image using median filtering and morpho math
  cv::Mat bin_image;
  imageprocessing::filter_image(merge_image_seg, bin_image);

  /*
   * Extract candidates (i.e., contours) and remove inconsistent candidates
   */

  std::vector< std::vector< cv::Point > > distorted_contours;
  imageprocessing::contours_extraction(bin_image, distorted_contours);

  /*
   * Correct the distortion for each contour
   */

  // Initialisation of the variables which will be returned after the distortion. These variables are linked with the transformation applied to correct the distortion
  std::vector< cv::Mat > rotation_matrix(distorted_contours.size());
  std::vector< cv::Mat > scaling_matrix(distorted_contours.size());
  std::vector< cv::Mat > translation_matrix(distorted_contours.size());
  for (unsigned int contour_idx = 0; contour_idx < distorted_contours.size(); contour_idx++) {
    rotation_matrix[contour_idx] = cv::Mat::eye(3, 3, CV_64F);
    scaling_matrix[contour_idx] = cv::Mat::eye(3, 3, CV_64F);
    translation_matrix[contour_idx] = cv::Mat::eye(3, 3, CV_64F);
  }

  // Correct the distortion 
  std::vector< std::vector< cv::Point2f > > undistorted_contours;
  imageprocessing::correction_distortion (distorted_contours, undistorted_contours, translation_matrix, rotation_matrix, scaling_matrix);

  // Normalise the contours to be inside a unit circle
  std::vector<double> factor_vector(undistorted_contours.size());
  std::vector< std::vector< cv::Point2f > > normalised_contours;
  initoptimisation::normalise_all_contours(undistorted_contours, normalised_contours, factor_vector);

  // Check the center mass for a contour
  cv::Point2f dum = initoptimisation::mass_center_discovery(input_image, translation_matrix[0], rotation_matrix[0], scaling_matrix[0], normalised_contours[0], factor_vector[0], 2);

  std::vector< cv::PointPolar2f > ct_pol(normalised_contours[0].size());
  initoptimisation::rotation_offset(normalised_contours[0]);

  // std::cout << dum << std::endl;

  // cv::Mat output_image = cv::Mat::zeros(bin_image.size(), CV_8U);
  // cv::Scalar color(255,255,255);
  // cv::drawContours(output_image, distorted_contours, -1, color, 0, 8);
  // cv::namedWindow("Window", CV_WINDOW_AUTOSIZE);
  // cv::imshow("Window", output_image);
  // cv::waitKey(0);

  end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end-start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);
 
  std::cout << "Finished computation at " << std::ctime(&end_time)
	    << "Elapsed time: " << elapsed_seconds.count()*1000 << "ms\n";

  return 0;
}
