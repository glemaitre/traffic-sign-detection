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

// own library
#include "SuperFormula.h"

// OpenCV library
#include <opencv2/opencv.hpp>

// Eigen library
#include <Eigen/Core>

#define THRESH_GRAD_RAD_DET 0.10
#define THRESH_BINARY 0.80

namespace optimisation {

// Structure for configuration regarding the optimisation
template<typename _Tp> class ConfigStruct_ {
public:
    // Class constructor
    // default constructor
    ConfigStruct_() { a = 1.0; b = 1.0; n1 = 2.0; n2 = 2.0; n3 = 2.0; p = 4.0; q = 1.0; theta_offset = 0.0; phi_offset = 0.0; x_offset = 0.0; y_offset = 0.0; z_offset = 0.0; }
    // constructor with initialisation
    ConfigStruct_(const _Tp& _a, const _Tp& _b, const _Tp& _n1, const _Tp& _n2, const _Tp& _n3, const _Tp& _p, const _Tp& _q, const _Tp& _theta_offset, const _Tp& _phi_offset, const _Tp& _x_offset, const _Tp& _y_offset, const _Tp& _z_offset) { a = _a; b = _b; n1 = _n1; n2 = _n2; n3 = _n3; p = _p; q = _q; theta_offset = _theta_offset; phi_offset = _phi_offset; x_offset = _x_offset; y_offset = _y_offset; z_offset = _z_offset; }

    // Operator =
    ConfigStruct_<_Tp>& operator=(const ConfigStruct_<_Tp>& cs) { a = cs.a; b = cs.b; n1 = cs.n1; n2 = cs.n2; n3 = cs.n3; p = cs.p; q = cs.q; theta_offset = cs.theta_offset; phi_offset = cs.phi_offset; x_offset = cs.x_offset; y_offset = cs.y_offset; z_offset = cs.z_offset; return *this; }

    // Operator <<
    friend std::ostream& operator<<(std::ostream& os, const ConfigStruct_<_Tp>& cs) { os
                << "a = "            << cs.a << "\n"
                << "b = "            << cs.b << "\n"
                << "n 1 = "          << cs.n1 << "\n"
                << "n 2 = "          << cs.n2 << "\n"
                << "n 3 = "          << cs.n3 << "\n"
                << "p = "            << cs.p << "\n"
                << "q = "            << cs.q << "\n"
                << "theta offset = " << cs.theta_offset << "\n"
                << "phi offset = "   << cs.phi_offset << "\n"
                << "x offset = "     << cs.x_offset << "\n"
                << "y offset = "     << cs.y_offset << "\n"
                << "z offset = "     << cs.z_offset << "\n";
                                                                                      return os; }

    // Class members
public:
    _Tp a;
    _Tp b;
    _Tp n1;
    _Tp n2;
    _Tp n3;
    _Tp p;
    _Tp q;
    _Tp theta_offset;
    _Tp phi_offset;
    _Tp x_offset;
    _Tp y_offset;
    _Tp z_offset;
};


typedef ConfigStruct_<int> ConfigStruct2i;
typedef ConfigStruct2i ConfigStruct;
typedef ConfigStruct_<float> ConfigStruct2f;
typedef ConfigStruct_<double> ConfigStruct2d;

// Function to make the optimisation
void gielis_optimisation(const std::vector< cv::Point2f >& contour, ConfigStruct2d& config_shape, Eigen::Vector4d& mean_err, Eigen::Vector4d& std_err);

// Reconstruction using the Gielis formula
void gielis_reconstruction(const ConfigStruct2d& config_shape, std::vector< cv::Point2f >& gielis_contour, const int number_points);
}
