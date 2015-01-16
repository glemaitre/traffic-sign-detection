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

// OpenCV library
#include <opencv2/opencv.hpp>

namespace mathutils {

  // Maximum between three values
  inline unsigned int get_maximum(const unsigned int& r, const unsigned int& g, const unsigned int& b) { return (r >= g) ? ((r >= b) ? r : b) : ((g >= b) ? g : b); }

  // Minimum between three values
  inline unsigned int get_minimum(const unsigned int& r, const unsigned int& g, const unsigned int& b) { return (r <= g) ? ((r <= b) ? r : b) : ((g <= b) ? g : b); }

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
    typedef _Tp value_type;
    // various constructors
    PointPolar_() { phi = 0; theta = 0; }
    PointPolar_(_Tp _phi, _Tp _theta) { phi = _phi; theta = _theta; }
       
    _Tp phi, theta; //< the point coordinates
  };

typedef PointPolar_<int> PointPolar2i;
typedef PointPolar2i PointPolar;
typedef PointPolar_<float> PointPolar2f;
typedef PointPolar_<double> PointPolar2d;
  
}

#endif /* MATH_UTILS_H_ */
