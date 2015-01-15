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

#ifndef COLORCONVERSION_H
#define COLORCONVERSION_H

// stl library
#include <vector>
#include <cmath>

// OpenCV library
#include <opencv2/opencv.hpp>

// own library
#include "math_utils.h"

namespace colorconversion {

  // Conversion from RGB to logarithm RB
  std::vector< cv::Mat > rgb_to_log_rb(cv::Mat rgb_image);

  // Conversion from RGB to IHLS
  cv::Mat convert_rgb_to_ihls(cv::Mat rgb_image);

  // Theta computation
  inline float retrieve_theta(unsigned int r, unsigned int g, unsigned int b) { return acos((r - (g * 0.5) - (b * 0.5)) / sqrtf((r * r) + (g * g) + (b * b) - (r * g) - (r * b) - (g * b))); }
  // Hue computation -- H = θ if B <= G -- H = 2 * pi − θ if B > G
  inline float retrieve_normalised_hue(unsigned int r, unsigned int g, unsigned int b) { return (b <= g) ? (retrieve_theta(r, g, b) * 255 / (2 * M_PI)) : (((2 * M_PI) - retrieve_theta(r, g, b)) * 255 / (2 * M_PI)); }
  // Luminance computation -- L = 0.210R + 0.715G + 0.072B
  inline float retrieve_luminance(unsigned int r, unsigned int g, unsigned int b) { return (0.210f * r) + (0.715f * g) + (0.072f * b); }
  // Saturation computation -- S = max(R, G, B) − min(R, G, B)
  inline float retrieve_saturation(unsigned int r, unsigned int g, unsigned int b) { return (mathutils::get_maximum(r, g, b) - mathutils::get_minimum(r, g, b)); }

}

#endif // COLORCONVERSION_H
