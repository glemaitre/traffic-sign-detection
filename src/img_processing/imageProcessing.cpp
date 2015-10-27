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

#include "imageProcessing.h"

// stl library
#include <vector>

namespace imageprocessing {

// Function to filter the image based on median filtering and morpho math
void filter_image(const cv::Mat& seg_image, cv::Mat& bin_image) {

    // Allocate the output
    bin_image = seg_image.clone();

    // Create the structuring element for the erosion and dilation
    cv::Mat struct_elt = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(4, 4));

    // Apply the dilation
    cv::dilate(bin_image, bin_image, struct_elt);
    // Threshold the image
    cv::threshold(bin_image, bin_image, 254, 255, CV_THRESH_BINARY);

    // Find the contours of the objects
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;
    cv::findContours(bin_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    // Filled the objects
    cv::Scalar color(255, 255, 255);
    cv::drawContours(bin_image, contours, -1, color, CV_FILLED, 8);

    // Apply some erosion on the destination image
    cv::erode(bin_image, bin_image, struct_elt);

    // Noise filtering via median filtering
    for (int i = 0; i < 5; ++i)
        cv::medianBlur(bin_image, bin_image, 5);

}

// Function to remove ill-posed contours
void removal_elt(std::vector< std::vector< cv::Point > >& contours, const cv::Size size_image, const long int areaRatio, const double lowAspectRatio, const double highAspectRatio) {

    for (auto it = contours.begin(); it != contours.end(); /*We remove some part of the vector - DO NOT INCREMENT HERE*/) {
        // Find a bounding box to compute around the contours
        const cv::Rect bound_rect = cv::boundingRect(cv::Mat(*it));
        // Compute the aspect ratio
        const double ratio = static_cast<double> (bound_rect.width) / static_cast<double> (bound_rect.height);
        const long int areaRegion = bound_rect.area();

        // Check the inconsistency
        if ((areaRegion < size_image.area() / areaRatio) || ((ratio > highAspectRatio) || (ratio < lowAspectRatio)))
            contours.erase(it);
        else
            ++it;
    }
}

// Compute the distance between the edge points (po and pf), with the current point pc
float distance(const cv::Point& po, const cv::Point& pf, const cv::Point& pc)
{
    // Cast into float to compute right distances
    const cv::Point2f po2f(static_cast<float> (po.x), static_cast<float> (po.y));
    const cv::Point2f pf2f(static_cast<float> (pf.x), static_cast<float> (pf.y));
    const cv::Point2f pc2f(static_cast<float> (pc.x), static_cast<float> (pc.y));

    // In this function, we will compute the altitude of the triangle form by the two points of the convex hull and the one of the contour.
    // It will allow us to remove points far of the convex conserving a degree of freedom

    // Compute the three length of each side of the triangle
    // a will be the base too
    const float a = std::sqrt(std::pow(pf2f.x - po2f.x, 2.00) + std::pow(pf2f.y - po2f.y, 2.00));
    // Compute the two other sides
    const float b = std::sqrt(std::pow(pc2f.x - po2f.x, 2.00) + std::pow(pc2f.y - po2f.y, 2.00));
    const float c = std::sqrt(std::pow(pf2f.x - pc2f.x, 2.00) + std::pow(pf2f.y - pc2f.y, 2.00));

    // Compute S which is the perimeter of the triangle divided by 2
    const float s = (a + b + c) / 2.00;
    // Compute the area of the triangle
    const float area = std::sqrt(s * (s - a) * (s - b) * (s - c));
    // Compute the altitude
    return 2.00f * area / a;
}

// Remove the inconsistent points inside each contour
void contours_thresholding(const std::vector< std::vector< cv::Point > >& hull_contours, const std::vector< std::vector< cv::Point > >& contours, std::vector< std::vector< cv::Point > >& final_contours, const float dist_threshold) {

    if (!final_contours.empty())
        final_contours.erase(final_contours.begin(), final_contours.end());

    cv::Point current_hull_point, contour_point, next_hull_point;

    // For each contour
    for (size_t contour_idx = 0; contour_idx < contours.size(); ++contour_idx) {
        int hull_idx = 0;
        current_hull_point = hull_contours[contour_idx][hull_idx];
        contour_point = contours[contour_idx][0];

        // Find a correspondences between the hull and the contours
        while (current_hull_point != contour_point) {
            hull_idx++;
            current_hull_point = hull_contours[contour_idx][hull_idx];
        }

        // Explore point by point of the hull in order to evaluate the consistency of a given contour point
        hull_idx = ((hull_idx - 1) + hull_contours[contour_idx].size()) % hull_contours[contour_idx].size();
        next_hull_point = hull_contours[contour_idx][hull_idx];

        // Vector to store the good contour point
        std::vector< cv::Point > good_contour;

        // Check each contour point
        for (size_t i = 0; i < contours[contour_idx].size(); ++i) {
            // Store the current point
            contour_point = contours[contour_idx][i];

            // If the contour point is near to the convex_hull edge add it to the output
            if (distance(current_hull_point, next_hull_point, contour_point) < dist_threshold)
                good_contour.push_back(contour_point);

            // If the explored point is the same than the next point of the convex hull, then change next convex hull to current convex hull point
            if (next_hull_point == contour_point) {
                current_hull_point = hull_contours[contour_idx][hull_idx];
                hull_idx = ((hull_idx - 1) + hull_contours[contour_idx].size() ) % hull_contours[contour_idx].size();
                next_hull_point = hull_contours[contour_idx][hull_idx];
            }
        }
        final_contours.push_back(good_contour);
    }
}

// Function to extract the contour with some denoising step
void contours_extraction(const cv::Mat& bin_image, std::vector< std::vector< cv::Point > >& final_contours) {

    // Allocate the needed element
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;

    // Extract the raw contours
    cv::findContours(bin_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Need to remove some of the contours based on aspect ratio inconsistancy
    // DO NOT FORGET THAT THERE IS SOME PARAMETERS REGARDING THE ASPECT RATIO
    removal_elt(contours, bin_image.size());

    // Extract the convex_hull for each contours in order to make some processing to finally extract the final contours
    std::vector< std::vector< cv::Point > > hull_contours(contours.size());

    // Find the convec hull for each contours
    auto it_hull = hull_contours.begin();
    for (auto it = contours.begin(); it != contours.end(); ++it, ++it_hull)
        cv::convexHull(cv::Mat(*it), (*it_hull), false);

    // Extract the contours
    // DEFAULT VALUE OF 2.0 PIXELS
    contours_thresholding(hull_contours, contours, final_contours);
}

// Function to make forward transformation -- INPUT CV::POINT
void forward_transformation_contour(const std::vector < cv::Point >& contour, std::vector< cv::Point2f >& output_contour, const cv::Mat& translation_matrix, const cv::Mat& rotation_matrix, const cv::Mat& scaling_matrix) {

    // Convert the contour into cv::Point2f
    std::vector< cv::Point2f > contour2f(contour.size());
    auto it_contour = contour.begin();
    for (auto it = contour2f.begin(); it != contour2f.end(); ++it, ++it_contour) {
        (*it).x = static_cast<float> ((*it_contour).x);
        (*it).y = static_cast<float> ((*it_contour).y);
    }

    // Call the overloaded function for float point
    forward_transformation_contour(contour2f, output_contour, translation_matrix, rotation_matrix, scaling_matrix);

}

// Function to make forward transformation -- INPUT CV::POINT2F
void forward_transformation_contour(const std::vector < cv::Point2f >& contour, std::vector< cv::Point2f >& output_contour, const cv::Mat& translation_matrix, const cv::Mat& rotation_matrix, const cv::Mat& scaling_matrix) {

    // Calculate the affine transformation using the rotation, scaling and translation matrices
    cv::Mat transform_matrix;
    transform_matrix = rotation_matrix * scaling_matrix * translation_matrix;

    // Convert the input contour into homogenous coordinates
    std::vector< cv::Point3f > homogeneous_contour;
    cv::convertPointsToHomogeneous(contour, homogeneous_contour);

    // Apply the transformation
    cv::Mat tmp_transformed_contour;
    tmp_transformed_contour = transform_matrix * cv::Mat(homogeneous_contour).reshape(1).t();

    // Convert back the point into Euclidean space
    cv::Mat output_matrix;
    cv::convertPointsFromHomogeneous(tmp_transformed_contour.t(), output_matrix);

    // Convert back into a vector
    if (!output_contour.empty()) {
        output_contour.erase(output_contour.begin(), output_contour.end());
        output_matrix.copyTo(output_contour);
    }
    else
        output_matrix.copyTo(output_contour);

}

// Function to make forward transformation -- INPUT CV::POINT
void forward_transformation_point(const cv::Point2f& point, cv::Point2f& output_point, const cv::Mat& translation_matrix, const cv::Mat& rotation_matrix, const cv::Mat& scaling_matrix) {

    // Calculate the affine transformation using the rotation, scaling and translation matrices
    cv::Mat transform_matrix;
    transform_matrix = rotation_matrix * scaling_matrix * translation_matrix;

    // Convert the input contour into homogenous coordinates
    cv::Point3f homogeneous_point(point.x, point.y, 1.0f);

    // Apply the transformation
    cv::Mat tmp_transformed_contour;
    tmp_transformed_contour = transform_matrix * cv::Mat(homogeneous_point).reshape(1);

    // Convert back the point into Euclidean space
    auto it = tmp_transformed_contour.begin<cv::Vec3f>();
    output_point = cv::Point2f((*it)[0], (*it)[1]);
}

// Function to make inverse transformation -- INPUT CV::POINT
void inverse_transformation_contour(const std::vector < cv::Point >& contour, std::vector< cv::Point2f >& output_contour, const cv::Mat& translation_matrix, const cv::Mat& rotation_matrix, const cv::Mat& scaling_matrix) {

    // Convert the contour into cv::Point2f
    std::vector< cv::Point2f > contour2f(contour.size());
    auto it_contour = contour.begin();
    for (auto it = contour2f.begin(); it != contour2f.end(); ++it, ++it_contour) {
        (*it).x = static_cast<float> ((*it_contour).x);
        (*it).y = static_cast<float> ((*it_contour).y);
    }

    inverse_transformation_contour(contour2f, output_contour, translation_matrix, rotation_matrix, scaling_matrix);

}

// Function to make inverse transformation -- INPUT CV::POINT2F
void inverse_transformation_contour(const std::vector < cv::Point2f >& contour, std::vector< cv::Point2f >& output_contour, const cv::Mat& translation_matrix, const cv::Mat& rotation_matrix, const cv::Mat& scaling_matrix) {

    // Calculate the affine transformation using the rotation, scaling and translation matrices
    cv::Mat transform_matrix;
    transform_matrix = rotation_matrix * scaling_matrix * translation_matrix;
    transform_matrix = transform_matrix.inv();

    // Convert the input contour into homogenous coordinates
    std::vector< cv::Point3f > homogeneous_contour;
    cv::convertPointsToHomogeneous(contour, homogeneous_contour);

    // Apply the transformation
    cv::Mat tmp_transformed_contour;
    tmp_transformed_contour = transform_matrix * cv::Mat(homogeneous_contour).reshape(1).t();

    // Convert back the point into Euclidean space
    cv::Mat output_matrix;
    cv::convertPointsFromHomogeneous(tmp_transformed_contour.t(), output_matrix);

    // Convert back into a vector
    if (!output_contour.empty()) {
        output_contour.erase(output_contour.begin(), output_contour.end());
        output_matrix.copyTo(output_contour);
    }
    else
        output_matrix.copyTo(output_contour);
}

// Function to correct the distortion of the contours
void correction_distortion (const std::vector< std::vector < cv::Point > >& contours, std::vector< std::vector < cv::Point2f > >& output_contours, std::vector< cv::Mat >& translation_matrix, std::vector< cv::Mat >& rotation_matrix, std::vector< cv::Mat >& scaling_matrix) {

    // Allocation of the ouput -- The type is not anymore integer but float
    if(!output_contours.empty()) {
        output_contours.erase(output_contours.begin(), output_contours.end());
        output_contours.resize(contours.size());
    }
    else
        output_contours.resize(contours.size());

    // Conversion into float point
    auto it_output_contour = output_contours.begin();
    for (auto it_contour = contours.begin(); it_contour != contours.end(); ++it_output_contour, ++it_contour) {
        for (auto it_contour_point = (*it_contour).begin(); it_contour_point != (*it_contour).end(); ++it_contour_point) {
            (*it_output_contour).push_back(cv::Point2f((*it_contour_point).x, (*it_contour_point).y));
        }
    }

    // Correct the distortion for each contour
    auto it_contour = contours.begin();
    auto it_translation_matrix = translation_matrix.begin();
    auto it_rotation_matrix = rotation_matrix.begin();
    auto it_scaling_matrix = scaling_matrix.begin();
    for (auto it_output_contour = output_contours.begin(); it_output_contour != output_contours.end(); it_contour++, it_output_contour++, ++it_translation_matrix, ++it_rotation_matrix, ++it_scaling_matrix) {

        // Compute the moments of each contour
        cv::Moments contour_moments = cv::moments((*it_output_contour));

        // Compute the mass center
        const float xbar = contour_moments.m10 / contour_moments.m00;
        const float ybar = contour_moments.m01 / contour_moments.m00;

        // Compute the second order central moment
        const float mu11p = contour_moments.mu11 / contour_moments.m00;
        const float mu20p = contour_moments.mu20 / contour_moments.m00;
        const float mu02p = contour_moments.mu02 / contour_moments.m00;

        // Compute the object orientation in order to determine the rotation matrix
        float contour_orientation;
        if (mu11p != 0)
            contour_orientation = 0.5 * std::atan((2 * mu11p) / (mu20p - mu02p));
        else
            contour_orientation = 0.0;

        // Compute the covariance matrix in order to determine scaling matrix
        cv::Mat covariance_matrix = cv::Mat::zeros(2, 2, CV_32F);
        covariance_matrix.at<float>(0, 0) = mu20p;
        covariance_matrix.at<float>(0, 1) = mu11p;
        covariance_matrix.at<float>(1, 0) = mu11p;
        covariance_matrix.at<float>(1, 1) = mu02p;
        // Compute eigenvalues and eigenvector
        cv::Mat eigen_value_matrix, eigen_vector_matrix;
        cv::eigen(covariance_matrix, eigen_value_matrix, eigen_vector_matrix);

        // Create the rotation matrix
        (*it_rotation_matrix).at<float>(0, 0) = std::cos(contour_orientation);
        (*it_rotation_matrix).at<float>(0, 1) = - std::sin(contour_orientation);
        (*it_rotation_matrix).at<float>(1, 0) = std::sin(contour_orientation);
        (*it_rotation_matrix).at<float>(1, 1) = std::cos(contour_orientation);

        // Create the scaling matrix
        if (contour_moments.mu20 > contour_moments.mu02) {
            (*it_scaling_matrix).at<float>(0, 0) = std::pow(eigen_value_matrix.at<float>(0, 0) * eigen_value_matrix.at<float>(1, 0), 0.25) / std::sqrt(eigen_value_matrix.at<float>(0, 0));
            (*it_scaling_matrix).at<float>(1, 1) = std::pow(eigen_value_matrix.at<float>(0, 0) * eigen_value_matrix.at<float>(1, 0), 0.25) / std::sqrt(eigen_value_matrix.at<float>(1, 0));
        }
        else {
            (*it_scaling_matrix).at<float>(0, 0) = std::pow(eigen_value_matrix.at<float>(0, 0) * eigen_value_matrix.at<float>(1, 0), 0.25) / std::sqrt(eigen_value_matrix.at<float>(1, 0));
            (*it_scaling_matrix).at<float>(1, 1) = std::pow(eigen_value_matrix.at<float>(0, 0) * eigen_value_matrix.at<float>(1, 0), 0.25) / std::sqrt(eigen_value_matrix.at<float>(0, 0));
        }

        // Create the translation matrix
        (*it_translation_matrix).at<float>(0, 2) = - xbar;
        (*it_translation_matrix).at<float>(1, 2) = - ybar;

        // Transform the contour using the previous found transformation
        forward_transformation_contour((*it_contour), (*it_output_contour), (*it_translation_matrix), (*it_rotation_matrix), (*it_scaling_matrix));
    }
}

}
