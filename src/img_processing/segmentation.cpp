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

#include "segmentation.h"

namespace segmentation {

/*
   * Segmentation of logarithmic chromatic image
   */
void seg_log_chromatic(const std::vector< cv::Mat >& log_image, cv::Mat& log_image_seg) {

    // Segment the image using the pre-defined threshold in the header of this file
    // Allocation of the original image
    log_image_seg.create(log_image[0].size(), CV_8UC1);

    for( size_t i = 0; i< log_image.size(); ++i){
        assert(log_image[i].type() == CV_32F);
    }

    // Make the segmentation by simple threholding
    for (int i = 0 ; i < log_image_seg.rows ; i++) {
        for (int j = 0 ; j < log_image_seg.cols ; j++) {
            const bool condR = (log_image[0].at<float>(i, j) > MINLOGRG)&&(log_image[0].at<float>(i, j) < MAXLOGRG);
            const bool condB = (log_image[1].at<float>(i, j) > MINLOGBG)&&(log_image[1].at<float>(i, j) < MAXLOGBG);
            /*----------- Red detection ----------*/
            log_image_seg.at<uchar>(i, j) = (condR && condB) ? 255 : 0;
            /*----------- Have to be done for blue too ------------*/
        }
    }
}

/*
   * Segmentation of IHLS image
   */
void seg_norm_hue(const cv::Mat& ihls_image, cv::Mat& nhs_image, const int& colour, int hue_max, int hue_min, int sat_min) {

    // Define the different thresholds
    if (colour == 2) {
        if (hue_max > 255 || hue_max < 0 || hue_min > 255 || hue_min < 0 || sat_min > 255 || sat_min < 0) {
            hue_min = R_HUE_MIN;
            hue_max = R_HUE_MAX;
            sat_min = R_SAT_MIN;
        }
    }
    else if (colour == 1) {
        hue_min = B_HUE_MIN;
        hue_max = B_HUE_MAX;
        sat_min = B_SAT_MIN;
    }
    else {
        hue_min = R_HUE_MIN;
        hue_max = R_HUE_MAX;
        sat_min = R_SAT_MIN;
    }

    // Check that the image has three channels
    CV_Assert(ihls_image.channels() == 3);

    // Create the ouput the image
    nhs_image.create(ihls_image.size(), CV_8UC1);

    // I put the if before for loops, to make the process faster.
    // Otherwise for each pixel it had to check this condition.
    // Nicer implementation could be to separate these two for loops in
    // two different functions, one for red and one for blue.
    if (colour == 1) {
        for (int i = 0; i < ihls_image.rows; ++i) {
            const uchar *ihls_data = ihls_image.ptr<uchar> (i);
            uchar *nhs_data = nhs_image.ptr<uchar> (i);
            for (int j = 0; j < ihls_image.cols; ++j) {
                uchar s = *ihls_data++;
                // Although l is not being used and we could have
                // replaced the next line with ihls_data++
                // but for the sake of readability, we left it as it it.
                uchar l = *ihls_data++;
                uchar h = *ihls_data++;
                *nhs_data++ = (B_CONDITION) ? 255 : 0;
            }
        }
    }
    else {
        for (int i = 0; i < ihls_image.rows; ++i) {
            const uchar *ihls_data = ihls_image.ptr<uchar> (i);
            uchar *nhs_data = nhs_image.ptr<uchar> (i);
            for (int j = 0; j < ihls_image.cols; ++j) {
                uchar s = *ihls_data++;
                // Although l is not being used and we could have
                // replaced the next line with ihls_data++
                // but for the sake of readability, we left it as it it.
                uchar l = *ihls_data++;
                uchar h = *ihls_data++;
                *nhs_data++ = (R_CONDITION) ? 255 : 0;
            }
        }
    }
}

}
