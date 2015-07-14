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

#pragma once

// stl library
#include <iostream>

// OpenCV library
#include <opencv2/opencv.hpp>

//math constants
#define EPSILON 1e-9
#define FLAT_TRIANGLE_AREA 0.0001

namespace mathutils {

// Maximum between three values
inline float get_maximum(const float& r, const float& g, const float& b) 
{ 
    return (r >= g) ? ((r >= b) ? r : b) : ((g >= b) ? g : b); 
}

// Minimum between three values
inline float get_minimum(const float& r, const float& g, const float& b) 
{ 
    return (r <= g) ? ((r <= b) ? r : b) : ((g <= b) ? g : b); 
}

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
    friend std::ostream& operator<<(std::ostream& os, const PointPolar_<_Tp>& pt) 
    { 
	os << "[" << pt.phi << ", " << pt.theta << "]"; 
	return os; 
    }

    _Tp phi, theta; //< the point coordinates
};

typedef PointPolar_<int> PointPolar2i;
typedef PointPolar2i PointPolar;
typedef PointPolar_<float> PointPolar2f;
typedef PointPolar_<double> PointPolar2d;

}
