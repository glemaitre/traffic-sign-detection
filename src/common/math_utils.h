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

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

// stl library
#include <iostream>

// OpenCV library
#include <opencv2/opencv.hpp>

//math constants
#define PI 3.14159265358979323846
#define ZERO 1e-9
#define EPSILON 1e-9
#define ITERATION_MAX 10
#define NEGATIVE -1
#define POSITIVE 1
#define FLAT_TRIANGLE_AREA 0.0001

namespace mathutils {

  // Maximum between three values
  inline float get_maximum(const float& r, const float& g, const float& b) { return (r >= g) ? ((r >= b) ? r : b) : ((g >= b) ? g : b); }

  // Minimum between three values
  inline float get_minimum(const float& r, const float& g, const float& b) { return (r <= g) ? ((r <= b) ? r : b) : ((g <= b) ? g : b); }

}

namespace cv {
  
  //////////////////////////////// PointPolar_ ////////////////////////////////
  /*!
    template 2D polar point class.
    The class defines a point in 2D polar space. Data type of the point coordinates is specified
    as a template parameter. There are a few shorter aliases available for user convenience.
  */
  template<typename _Tp> class PointPolar_ {
  public:

    //typedef _Tp value_type;

    // various constructors
    PointPolar_() { phi = 0; theta = 0; }
    PointPolar_(const PointPolar_<_Tp>& pt) { phi = pt.phi; theta = pt.theta; }
    PointPolar_(_Tp _phi, _Tp _theta) { phi = _phi; theta = _theta; }

    // function euclidean to polar coordinates
    void eucl_to_polar(const _Tp& _x, const _Tp& _y) {
      // Convert Euclidean coordinate to polar coordinate
      theta = std::atan2(_y, _x);
      if (theta < 0 ) 
	theta += 2 * M_PI;
      phi = std::sqrt(std::pow(_y, 2.00) + std::pow(_x, 2.00));
    }
    void eucl_to_polar(const cv::Point_<_Tp>& _pt) { 
      // Convert Euclidean coordinate to polar coordinate
      theta = std::atan2(_pt.y, _pt.x);
      if (theta < 0 ) 
	theta += 2 * M_PI;
      phi = std::sqrt(std::pow(_pt.y, 2.00) + std::pow(_pt.x, 2.00));
    }

    // function polar to euclidean coordinates
    cv::Point_<_Tp> polar_to_eucl() {
      // Convert polar to euclidean coordinates
      return cv::Point(phi * std::cos(theta), phi * std::sin(theta));
    }
    void polar_to_eucl(_Tp& _x,_Tp& _y) {
      // Convert polar to euclidean coordinates
      _x = phi * std::cos(theta);
      _y = phi * std::sin(theta);
    }

    // Overload of some operator
    bool operator<(const PointPolar_<_Tp> &pt) const { return theta < pt.theta; }
    bool operator>(const PointPolar_<_Tp> &pt) const { return theta > pt.theta; }
    friend std::ostream& operator<<(std::ostream& os, const PointPolar_<_Tp>& pt) { os << "[" << pt.phi << ", " << pt.theta << "]"; return os; }

    _Tp phi, theta; //< the point coordinates
  };

  typedef PointPolar_<int> PointPolar2i;
  typedef PointPolar2i PointPolar;
  typedef PointPolar_<float> PointPolar2f;
  typedef PointPolar_<double> PointPolar2d;
  
}

#endif /* MATH_UTILS_H_ */
