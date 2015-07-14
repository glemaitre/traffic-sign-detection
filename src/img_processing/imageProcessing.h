/*
By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install,
copy or use the software.


                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2015,
      Guillaume Lemaitre (g.lemaitre58@gmail.com),
      Johan Massich (mailsik@gmail.com),
      Gerard Bahi (zomeck@gmail.com),
      Yohan Fougerolle (Yohan.Fougerolle@u-bourgogne.fr).
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall copyright holders or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#pragma once
// OpenCV library
#include <opencv2/opencv.hpp>

namespace imageprocessing {

// Filter the binary image using morpho math and median filtering
void filter_image(const cv::Mat& seg_image, cv::Mat& bin_image);

// Elimination of objects based on inconsistent aspects ratio and areas
void removal_elt(std::vector< std::vector< cv::Point > >& contours, const cv::Size size_image, const long int areaRatio = 1500, const double lowAspectRatio = 0.5, const double highAspectRatio = 1.3);

// Compute the distance between the edge points (po and pf), with th current point pc
float distance(const cv::Point& po, const cv::Point& pf, const cv::Point& pc);

// Remove the inconsitent points inside each contour
void contours_thresholding(const std::vector< std::vector< cv::Point > >& hull_contours, const std::vector< std::vector< cv::Point > >& contours, std::vector< std::vector< cv::Point > >& final_contours, const float dist_threshold = 2.0);

// Function to extract the contour with some denoising step
void contours_extraction(const cv::Mat& bin_image, std::vector< std::vector< cv::Point > >& final_contours);

// Function to make forward transformation -- INPUT CV::POINT
void forward_transformation_contour(const std::vector < cv::Point >& contour, std::vector< cv::Point2f >& output_contour, const cv::Mat& translation_matrix = cv::Mat::eye(3, 3, CV_32F), const cv::Mat& rotation_matrix = cv::Mat::eye(3, 3, CV_32F), const cv::Mat& scaling_matrix = cv::Mat::eye(3, 3, CV_32F));

// Function to make forward transformation -- INPUT CV::POINT2F
void forward_transformation_contour(const std::vector < cv::Point2f >& contour, std::vector< cv::Point2f >& output_contour, const cv::Mat& translation_matrix = cv::Mat::eye(3, 3, CV_32F), const cv::Mat& rotation_matrix = cv::Mat::eye(3, 3, CV_32F), const cv::Mat& scaling_matrix = cv::Mat::eye(3, 3, CV_32F));

// Function to make forward transformation -- INPUT CV::POINT2F
void forward_transformation_point(const cv::Point2f& point, cv::Point2f& output_point, const cv::Mat& translation_matrix = cv::Mat::eye(3, 3, CV_32F), const cv::Mat& rotation_matrix = cv::Mat::eye(3, 3, CV_32F), const cv::Mat& scaling_matrix = cv::Mat::eye(3, 3, CV_32F));

// Function to make inverse transformation -- INPUT CV::POINT
void inverse_transformation_contour(const std::vector < cv::Point >& contour, std::vector< cv::Point2f >& output_contour, const cv::Mat& translation_matrix = cv::Mat::eye(3, 3, CV_32F), const cv::Mat& rotation_matrix = cv::Mat::eye(3, 3, CV_32F), const cv::Mat& scaling_matrix = cv::Mat::eye(3, 3, CV_32F));

// Function to make inverse transformation -- INPUT CV::POINT2F
void inverse_transformation_contour(const std::vector < cv::Point2f >& contour, std::vector< cv::Point2f >& output_contour, const cv::Mat& translation_matrix = cv::Mat::eye(3, 3, CV_32F), const cv::Mat& rotation_matrix = cv::Mat::eye(3, 3, CV_32F), const cv::Mat& scaling_matrix = cv::Mat::eye(3, 3, CV_32F));

// Fuction to remove the distortion of each contour
void correction_distortion (const std::vector< std::vector < cv::Point > >& contours, std::vector< std::vector < cv::Point2f > >& output_contours, std::vector< cv::Mat >& translation_matrix, std::vector< cv::Mat >& rotation_matrix, std::vector< cv::Mat >& scaling_matrix);

}
