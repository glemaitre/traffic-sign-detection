// stl library
#include <string>
#include <iostream>

// OpenCV library
#include <opencv2/opencv.hpp>

// our own code
#include "ihls.h"
#include "nhs.h"

int main(int argc, char *argv[]) {

  // Chec the number of arguments
  if (argc != 2) {
    std::cout << "********************************" << std::endl;
    std::cout << "Usage of the code: ./traffic-sign-detection imageFileName.extension" << std::endl;
    std::cout << "********************************" << std::endl;

    return -1;
  }

  // Read the input image - convert char* to string
  std::string inputFilename(argv[1]);

  // Read the input image
  cv::Mat inputImage = cv::imread(inputFilename);

  // Check that the image has been opened 
  if (!inputImage.data) {
    std::cout << "Error to read the image. Check ''cv::imread'' function of OpenCV" << std::endl;
  }
  // Check that the image read is a 3 channels image
  cv::asset(inputImage.channels() == 3)

  // Conversion of the rgb image in ihls color space
  cv::Mat ihlsImage = convert_rgb_to_ihls(inputImage);

  // ONE PARAMETER TO CONSIDER - COLOR OF THE TRAFFIC SIGN TO DETECT - RED VS BLUE
  int nhsMode = 0; // nhsMode == 0 -> red segmentation / nhsMode == 1 -> blue segmentation

  // Segmentation of the image based on the hue
  cv::Mat nhsImage = convert_ihls_to_nhs(ihlsImage, nhsMode);
  
  
    // First we have to convert the bgr image into log chromatic values
    std::vector< cv::Mat >  = rgb2logRB(originalImage);

    // Then make the segmentation of the log image


  return 0;
}
