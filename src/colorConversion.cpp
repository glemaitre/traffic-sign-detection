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

#include "colorConversion.h"

namespace colorconversion {

  // Function to convert an RGB image (uchar) to log chromatic format (double)
  void rgb_to_log_rb(const cv::Mat& rgbImage, std::vector< cv::Mat >& log_chromatic_image) {

    // Allocate the output - Format: double with two channels
    cv::Mat log_chromatic_r = cv::Mat::zeros(rgbImage.size(), CV_64F);
    cv::Mat log_chromatic_b = cv::Mat::zeros(rgbImage.size(), CV_64F);
    if (!log_chromatic_image.empty())
      log_chromatic_image.erase(log_chromatic_image.begin(), log_chromatic_image.end());
    
    // Split the channels rgb
    std::vector< cv::Mat > rgbVector;
    cv::split(rgbImage, rgbVector);

    // Compute the conversion
    for (int i = 0 ; i < rgbImage.rows ; i++) {
      for (int j = 0 ; j < rgbImage.cols ; j++) {

	// The image in opencv are encoded in BGR and not RGB
	double blue = (double) rgbVector[0].at< uchar >(i, j);
	double green = (double) rgbVector[1].at< uchar >(i, j);
	double red = (double) rgbVector[2].at< uchar >(i, j);

	// In order to avoid any division by zero and infinity with the ln function
	if (blue == 0) blue = 1.00;
	if (green == 0) green = 1.00;
	if (red == 0) red = 1.00;

	// Compute the log chromatic red
	log_chromatic_r.at<double>(i, j) = log (red / green);

	// Compute the log chromatic blue
	log_chromatic_b.at<double>(i, j) = log (blue / green);
      }
    }

    log_chromatic_image.push_back(log_chromatic_r);
    log_chromatic_image.push_back(log_chromatic_b);
  }

  // Conversion from RGB to IHLS
  void convert_rgb_to_ihls(const cv::Mat& rgb_image, cv::Mat& ihls_image) {
    
    // Check the that the image has three channels
    CV_Assert(rgb_image.channels() == 3);

    // Create the output image if needed
    ihls_image.create(rgb_image.size(), CV_8UC3);

    // Make the conversion pixel by pixel
    for (int i = 0; i < rgb_image.rows; ++i) {
	const uchar* rgb_data = rgb_image.ptr<uchar> (i);
	uchar* ihls_data = ihls_image.ptr<uchar> (i);

	for (int j = 0; j < rgb_image.cols; ++j) {
	    unsigned int b = *rgb_data++;
	    unsigned int g = *rgb_data++;
	    unsigned int r = *rgb_data++;
	    *ihls_data++ = (uchar) retrieve_saturation(r, g, b);
	    *ihls_data++ = (uchar) retrieve_luminance(r, g, b);
	    *ihls_data++ = (uchar) retrieve_normalised_hue(r, g, b);
	}
    }
  }

}
