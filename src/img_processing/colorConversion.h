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

// own library
#include <common/math_utils.h>

// stl library
#include <vector>
#include <cmath>

// OpenCV library
#include <opencv2/opencv.hpp>

namespace colorconversion {

// Conversion from RGB to logarithm RB
void rgb_to_log_rb(const cv::Mat& rgb_image, std::vector< cv::Mat >& log_chromatic_image);

// Conversion from RGB to IHLS
void convert_rgb_to_ihls(const cv::Mat& rgb_image, cv::Mat& ihls_image);

// Theta computation
inline float retrieve_theta(const float& r, const float& g, const float& b) { return acos((r - (g * 0.5) - (b * 0.5)) / sqrtf((r * r) + (g * g) + (b * b) - (r * g) - (r * b) - (g * b))); }
// Hue computation -- H = θ if B <= G -- H = 2 * pi − θ if B > G
inline float retrieve_normalised_hue(const float& r, const float& g, const float& b) { return (b <= g) ? (retrieve_theta(r, g, b) * 255.f / (2.f * M_PI)) : (((2.f * M_PI) - retrieve_theta(r, g, b)) * 255.f / (2.f * M_PI)); }
// Luminance computation -- L = 0.210R + 0.715G + 0.072B
inline float retrieve_luminance(const float& r, const float& g, const float& b) { return (0.210f * r) + (0.715f * g) + (0.072f * b); }
// Saturation computation -- S = max(R, G, B) − min(R, G, B)
inline float retrieve_saturation(const float& r, const float& g, const float& b) { return (mathutils::get_maximum(r, g, b) - mathutils::get_minimum(r, g, b)); }

}

