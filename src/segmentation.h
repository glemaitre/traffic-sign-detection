#ifndef SEGMENTATION_H
#define SEGMENTATION_H

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
  cv::Mat seg_log_chromatic(std::vector< cv::Mat > log_image);

  // Segmentation of normalised hue
  cv::Mat seg_norm_hue(cv::Mat ihls_image, int colour = 0, int hue_max = R_HUE_MAX, int hue_min = R_HUE_MIN, int sat_min = R_SAT_MIN);

}

#endif // SEGMENTATION_H
