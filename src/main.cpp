// stl library
#include <string>
#include <iostream>

// OpenCV library
#include <opencv2/opencv.hpp>

// our own code
#include "segmentation.h"
#include "colorConversion.h"

int main(int argc, char *argv[]) {

  // Chec the number of arguments
  if (argc != 2) {
    std::cout << "********************************" << std::endl;
    std::cout << "Usage of the code: ./traffic-sign-detection imageFileName.extension" << std::endl;
    std::cout << "********************************" << std::endl;

    return -1;
  }

  // Read the input image - convert char* to string
  std::string input_filename(argv[1]);

  // Read the input image
  cv::Mat input_image = cv::imread(input_filename);

  // Check that the image has been opened 
  if (!input_image.data) {
    std::cout << "Error to read the image. Check ''cv::imread'' function of OpenCV" << std::endl;
    return -1;
  }
  // Check that the image read is a 3 channels image
  CV_Assert(input_image.channels() == 3);

  // Conversion of the rgb image in ihls color space
    cv::Mat ihls_image = colorconversion::convert_rgb_to_ihls(input_image);

  // Segmentation of the IHLS and more precisely of the normalised hue channel 
  // ONE PARAMETER TO CONSIDER - COLOR OF THE TRAFFIC SIGN TO DETECT - RED VS BLUE
  int nhs_mode = 0; // nhs_mode == 0 -> red segmentation / nhs_mode == 1 -> blue segmentation
  cv::Mat nhs_image_seg = segmentation::seg_norm_hue(ihls_image, nhs_mode);
  
  // Conversion from RGB to logarithmic chromatic red and blue
  std::vector< cv::Mat > log_image = colorconversion::rgb_to_log_rb(input_image);

  // Segmentation of the log chromatic image
  cv::Mat log_image_seg = segmentation::seg_log_chromatic(log_image);


  return 0;
}
