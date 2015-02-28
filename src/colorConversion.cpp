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
    cv::Mat log_chromatic_r = cv::Mat::zeros(rgbImage.size(), CV_32F);
    cv::Mat log_chromatic_b = cv::Mat::zeros(rgbImage.size(), CV_32F);
    if (!log_chromatic_image.empty())
        log_chromatic_image.erase(log_chromatic_image.begin(), log_chromatic_image.end());
    
    // Split the channels rgb
    //std::vector< cv::Mat > rgbVector;

    const int blockIter = 64;
    const int Niiter = std::max(1, int(std::ceil(float(rgbImage.rows)/blockIter)));
    const int Njiter = std::max(1, int(std::ceil(float(rgbImage.cols)/blockIter)));

    for (int iit = 0; iit <= Niiter; ++iit)
    {
        for (int i = iit*blockIter ; i < iit*(blockIter+1); i++) {

            if(i>=rgbImage.rows)
            {
                break;
            }

            for (int jit = 0; jit <= Njiter; ++jit)
            {
                for (int j = jit*blockIter; j < jit*(blockIter+1); j++) {

                    if(j>=rgbImage.cols)
                    {
                        break;
                    }

                    // The image in opencv are encoded in BGR and not RGB
                    const cv::Vec3b px = rgbImage.at<cv::Vec3b>(i, j);

                    // Do not divide by zero
                    // Compute the log chromatic red
                    const float division = 1.0f / static_cast<float> (px[1] == 0 ? px[1] + 1 : px[1]);
                    log_chromatic_r.at<float>(i, j) = std::log(static_cast<float> (px[2])*division);

                    // Compute the log chromatic blue
                    log_chromatic_b.at<float>(i, j) = std::log(static_cast<float> (px[0])*division);
                }
            }
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
    // ihls_image.create(rgb_image.size(), CV_8UC3);
    ihls_image = rgb_image.clone();

    for (auto it = ihls_image.begin<cv::Vec3b>(); it != ihls_image.end<cv::Vec3b>(); ++it) {
        const cv::Vec3b bgr = (*it);
        (*it)[0] = static_cast<uchar> (retrieve_saturation(static_cast<float> (bgr[2]), static_cast<float> (bgr[1]), static_cast<float> (bgr[0])));
        (*it)[1] = static_cast<uchar> (retrieve_luminance(static_cast<float> (bgr[2]), static_cast<float> (bgr[1]), static_cast<float> (bgr[0])));
        (*it)[2] = static_cast<uchar> (retrieve_normalised_hue(static_cast<float> (bgr[2]), static_cast<float> (bgr[1]), static_cast<float> (bgr[0])));
    }
}

}
