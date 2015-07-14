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

// stl library
#include <vector>

// OpenCV library
#include <opencv2/opencv.hpp>

/* Definition for log segmentation */
// To segment red traffic signs
#define MINLOGRG 0.5
#define MAXLOGRG 2.1
// To segment blue traffic signs
#define MINLOGBG -0.9
#define MAXLOGBG 0.8

/* Definition for ihls segmentation */
// To segment red traffic signs
#define R_HUE_MAX 15 // R_HUE_MAX 11
#define R_HUE_MIN 240
#define R_SAT_MIN 25 // R_SAT_MIN 30
#define R_CONDITION (h < hue_max || h > hue_min) && s > sat_min
// To segment blue traffic signs
#define B_HUE_MAX 163
#define B_HUE_MIN 134
#define B_SAT_MIN 39 // B_SAT_MIN 20
#define B_CONDITION (h < hue_max && h > hue_min) && s > sat_min

namespace segmentation {

// Segmentation of logarithmic chromatic images
void seg_log_chromatic(const std::vector< cv::Mat >& log_image, cv::Mat& log_image_seg);

// Segmentation of normalised hue
void seg_norm_hue(const cv::Mat& ihls_image, cv::Mat& nhs_image, const int& colour = 0, int hue_max = R_HUE_MAX, int hue_min = R_HUE_MIN, int sat_min = R_SAT_MIN);

}
