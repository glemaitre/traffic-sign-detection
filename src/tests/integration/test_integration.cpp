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

// our own code
#include <img_processing/segmentation.h>
#include <img_processing/colorConversion.h>
#include <img_processing/imageProcessing.h>
#include <img_processing/contour.h>
#include <optimization/smartOptimisation.h>
#include <common/math_utils.h>


#include <iostream>

// OpenCV library
#include <opencv2/opencv.hpp>

// Eigen library
#include <Eigen/Core>

#include <gtest/gtest.h>

//TODO: This probably should be just regression tests...
//TODO: clean up code, a lot of data is not used, also find proper GT no hardcoded values
//this is a copy of the main app to do automatic testing using the test data of the repository
TEST(integration, realDataOctogonal17)
{

    std::string input_filename(TEST_DATA_DIR);
    input_filename.append("/octogonal0017.jpg");

    // Read the input image
    cv::Mat input_image = cv::imread(input_filename);

    // Check that the image has been opened
    ASSERT_TRUE( input_image.data != NULL);

    // Check that the image read is a 3 channels image
    GTEST_ASSERT_EQ(input_image.channels(), 3);

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
    cv::Mat nhs_image_seg_red;
    segmentation::seg_norm_hue(ihls_image, nhs_image_seg_red, nhs_mode);
    //nhs_mode = 1; // nhs_mode == 0 -> red segmentation / nhs_mode == 1 -> blue segmentation
    //cv::Mat nhs_image_seg_blue;
    cv::Mat nhs_image_seg_blue = nhs_image_seg_red.clone();
    //segmentation::seg_norm_hue(ihls_image, nhs_image_seg_blue, nhs_mode);
    // Segmentation of the log chromatic image
    // TODO - DEFINE THE THRESHOLD FOR THE BLUE TRAFFIC SIGN. FOR NOW WE AVOID THE PROCESSING FOR BLUE SIGN AND LET ONLY THE OTHER METHOD TO TAKE CARE OF IT.
    cv::Mat log_image_seg;
    segmentation::seg_log_chromatic(log_image, log_image_seg);

    /*
     * Merging and filtering of the previous segmentation
     */

    // Merge the results of previous segmentation using an OR operator
    // Pre-allocation of an image by cloning a previous image
    cv::Mat merge_image_seg_with_red = nhs_image_seg_red.clone();
    cv::Mat merge_image_seg = nhs_image_seg_blue.clone();
    cv::bitwise_or(nhs_image_seg_red, log_image_seg, merge_image_seg_with_red);
    cv::bitwise_or(nhs_image_seg_blue, merge_image_seg_with_red, merge_image_seg);

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
        rotation_matrix[contour_idx] = cv::Mat::eye(3, 3, CV_32F);
        scaling_matrix[contour_idx] = cv::Mat::eye(3, 3, CV_32F);
        translation_matrix[contour_idx] = cv::Mat::eye(3, 3, CV_32F);
    }

    // Correct the distortion
    std::vector< std::vector< cv::Point2f > > undistorted_contours;
    imageprocessing::correction_distortion(distorted_contours, undistorted_contours, translation_matrix, rotation_matrix, scaling_matrix);

    // Normalise the contours to be inside a unit circle
    std::vector<double> factor_vector(undistorted_contours.size());
    std::vector< std::vector< cv::Point2f > > normalised_contours;
    initopt::normalise_all_contours(undistorted_contours, normalised_contours, factor_vector);

    std::vector< std::vector< cv::Point2f > > detected_signs_2f(normalised_contours.size());
    std::vector< std::vector< cv::Point > > detected_signs(normalised_contours.size());

    std::vector<optimisation::ConfigStruct_<double> > massCenters(normalised_contours.size());
    // For each contours
    for (unsigned int contour_idx = 0; contour_idx < normalised_contours.size(); contour_idx++) {

        // For each type of traffic sign
        /*
       * sign_type = 0 -> nb_edges = 3;  gielis_sym = 6; radius
       * sign_type = 1 -> nb_edges = 4;  gielis_sym = 4; radius
       * sign_type = 2 -> nb_edges = 12; gielis_sym = 4; radius
       * sign_type = 3 -> nb_edges = 8;  gielis_sym = 8; radius
       * sign_type = 4 -> nb_edges = 3;  gielis_sym = 6; radius / 2
       */

        optimisation::ConfigStruct_<double> final_config;
        double best_fit = std::numeric_limits<double>::infinity();
        //int type_sign_to_keep = 0;
        for (int sign_type = 0; sign_type < 5; sign_type++) {

            // Check the center mass for a contour
            cv::Point2f mass_center = initopt::mass_center_discovery(input_image, translation_matrix[contour_idx],
                                                                     rotation_matrix[contour_idx], scaling_matrix[contour_idx],
                                                                     normalised_contours[contour_idx], factor_vector[contour_idx], sign_type);

            // Find the rotation offset
            double rot_offset = initopt::rotation_offset(normalised_contours[contour_idx]);

            // Declaration of the parameters of the gielis with the default parameters
            optimisation::ConfigStruct_<double> contour_config;
            // Set the number of symmetry
            int gielis_symmetry = 0;
            switch (sign_type) {
            case 0:
                gielis_symmetry = 6;
                break;
            case 1:
                gielis_symmetry = 4;
                break;
            case 2:
                gielis_symmetry = 4;
                break;
            case 3:
                gielis_symmetry = 8;
                break;
            case 4:
                gielis_symmetry = 6;
                break;
            }
            contour_config.p = gielis_symmetry;
            // Set the rotation matrix
            contour_config.theta_offset = rot_offset;
            // Set the mass center
            contour_config.x_offset = mass_center.x;
            contour_config.y_offset = mass_center.y;

            // Go for the optimisation
            Eigen::Vector4d mean_err(0,0,0,0), std_err(0,0,0,0);
            optimisation::gielis_optimisation(normalised_contours[contour_idx], contour_config, mean_err, std_err);

            mean_err = mean_err.cwiseAbs();
            double err_fit = mean_err.sum();

            if (err_fit < best_fit) {
                best_fit = err_fit;
                final_config = contour_config;
                //type_sign_to_keep = sign_type;
            }
        }

        // Reconstruct the contour
        std::cout << "Contour #" << contour_idx << ":\n" << final_config << std::endl;
        std::vector< cv::Point2f > gielis_contour;
        int nb_points = 1000;
        optimisation::gielis_reconstruction(final_config, gielis_contour, nb_points);
        std::vector< cv::Point2f > denormalised_gielis_contour;
        initopt::denormalise_contour(gielis_contour, denormalised_gielis_contour, factor_vector[contour_idx]);
        std::vector< cv::Point2f > distorted_gielis_contour;
        imageprocessing::inverse_transformation_contour(denormalised_gielis_contour, distorted_gielis_contour,
                                                        translation_matrix[contour_idx], rotation_matrix[contour_idx], scaling_matrix[contour_idx]);

        // Transform to cv::Point to show the results
        std::vector< cv::Point > distorted_gielis_contour_int(distorted_gielis_contour.size());
        for (unsigned int i = 0; i < distorted_gielis_contour.size(); i++) {
            distorted_gielis_contour_int[i].x = (int) std::round(distorted_gielis_contour[i].x);
            distorted_gielis_contour_int[i].y = (int) std::round(distorted_gielis_contour[i].y);
        }

        detected_signs_2f[contour_idx] = distorted_gielis_contour;
        detected_signs[contour_idx] = distorted_gielis_contour_int;

        massCenters[contour_idx] = final_config;
    }

    //only 1 traffic sign in this image
    GTEST_ASSERT_EQ(massCenters.size(), 1);

    const float errThresh = 1e10-4;

    GTEST_ASSERT_LE(std::abs(massCenters[0].x_offset - 0.0077245), errThresh);
    GTEST_ASSERT_LE(std::abs(massCenters[0].y_offset - 0.0f), errThresh);
    GTEST_ASSERT_LE(std::abs(massCenters[0].z_offset), errThresh);
    GTEST_ASSERT_LE(std::abs(massCenters[0].a - 0.88557), errThresh);
    GTEST_ASSERT_LE(std::abs(massCenters[0].b - 0.869246), errThresh);
    GTEST_ASSERT_LE(std::abs(massCenters[0].theta_offset - 0.738422), errThresh);
    GTEST_ASSERT_LE(std::abs(massCenters[0].phi_offset - 0.0f), errThresh);
}

