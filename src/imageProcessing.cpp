#include "imageProcessing.h"

// stl library
#include <vector>

namespace imageprocessing {

  // Function to filter the image based on median filtering and morpho math
  cv::Mat filter_image(cv::Mat seg_image) {
    
    // Create the structuring element for the erosion and dilation
    cv::Mat struct_elt = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(4, 4));

    // Apply the dilation
    cv::dilate(seg_image, seg_image, struct_elt);
    // Threshold the image
    cv::threshold(seg_image, seg_image, 254, 255, CV_THRESH_BINARY);

    // Find the contours of the objects
    std::vector< std::vector< cv::Point > > contours;
    cv::vector< cv::Vec4i > hierarchy;
    cv::findContours(seg_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    // Filled the objects
    cv::Scalar color(255, 255, 255);
    cv::drawContours(seg_image, contours, -1, color, CV_FILLED, 8);
    
    // Apply some erosion on the destination image
    cv::erode(seg_image, seg_image, struct_elt);

    // Noise filtering via median filtering
    for (int i=0; i < 5; i++)
      cv::medianBlur(seg_image, seg_image, 5);

    return seg_image;
  }

  // Function to remove ill-posed contours
  //cv::Mat -- CAN RETURN A CV::MAT TO OBSERVE THE IMAGE - NOT NECESSARY IF YOU WANT TO SPEED UP THE APPLICATION
  void removal_elt(std::vector< std::vector< cv::Point > > &contours, std::vector< cv::Vec4i > hierarchy, cv::Size size_image, long int areaRatio, double lowAspectRatio, double highAspectRatio) {

    // For each contours   
    for (unsigned int i=0; i < contours.size(); i++) {

      // Find a bounding box to compute around the contours
      cv::Rect bound_rect = cv::boundingRect(cv::Mat(contours[i]));
      // Compute the aspect ratio
      double ratio = (double) bound_rect.width / (double) bound_rect.height;
      long int areaRegion = bound_rect.area();

      // Check the inconsistency
      if ((areaRegion < size_image.area() / areaRatio) || ((ratio > highAspectRatio) || (ratio < lowAspectRatio))) {
	contours.erase(contours.begin() + i);
	hierarchy.erase(hierarchy.begin() + i);
	i--;
      }
    }

    // // Return an image with the contours 
    // // Allocate the output image
    // cv::Mat output_image = cv::Mat::zeros(size_image, CV_8U);
    // // Draw the contours on the image
    // cv::Scalar color(255, 255, 255);
    // cv::drawContours(output_image, contours, -1, color, CV_FILLED, 8);

    // return output_image;
  }

  // Compute the distance between the edge points (po and pf), with the current point pc
  float distance(cv::Point po, cv::Point pf, cv::Point pc)
  {
    // Cast into float to compute right distances
    float pox = (float) po.x;
    float poy = (float) po.y;
    float pfx = (float) pf.x;
    float pfy = (float) pf.y;
    float pcx = (float) pc.x;
    float pcy = (float) pc.y;

    // In this function, we will compute the altitude of the triangle form by the two points of the convex hull and the one of the contour.
    // It will allow us to remove points far of the convex conserving a degree of freedom

    // Compute the three length of each side of the triangle
    // a will be the base too
    float a = sqrt(pow(pfx - pox, 2.00) + pow(pfy - poy, 2.00));
    // Compute the two other sides
    float b = sqrt(pow(pcx - pox, 2.00) + pow(pcy - poy, 2.00));
    float c = sqrt(pow(pfx - pcx, 2.00) + pow(pfy - pcy, 2.00));

    // Compute S which is the perimeter of the triangle divided by 2
    float s = (a + b + c) / 2.00;

    // Compute the area of the triangle
    float area = sqrt(s*(s - a)*(s - b)*(s-c));

    // Compute the altitude
    float altitude = 2.00 * area / a;

    return altitude;
  }

  // Remove the inconsistent points inside each contour
  std::vector< std::vector< cv::Point > > contours_thresholding(std::vector< std::vector< cv::Point > > hull_contours, std::vector< std::vector< cv::Point > > contours, float dist_threshold) {
   
    std::vector< std::vector< cv::Point > > final_contours;

    cv::Point current_hull_point, contour_point, next_hull_point;

    // For each contour
    for (unsigned int contour_idx = 0; contour_idx < contours.size(); contour_idx++) {
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
      for (unsigned int i = 0; i < contours[contour_idx].size(); i++ ) {
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
    return final_contours;
  }

  // Function to extract the contour with some denoising step
  std::vector< std::vector< cv::Point > > contours_extraction(cv::Mat bin_image) {

    // Allocate the needed element
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;

    // Extract the raw contours
    cv::findContours(bin_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Need to remove some of the contours based on aspect ratio inconsistancy
    // DO NOT FORGET THAT THERE IS SOME PARAMETERS REGARDING THE ASPECT RATIO
    removal_elt( contours, hierarchy, bin_image.size());

    // Extract the convex_hull for each contours in order to make some processing to finally extract the final contours
    std::vector< std::vector< cv::Point > > hull_contours(contours.size());

    // Find the convec hull for each contours
    for (unsigned int i = 0; i < contours.size(); i++)
      cv::convexHull(cv::Mat(contours[i]), hull_contours[i], false);
    
    // Extract the contours
    // DEFAULT VALUE OF 2.0 PIXELS
    return contours_thresholding(hull_contours, contours);
  }

}
