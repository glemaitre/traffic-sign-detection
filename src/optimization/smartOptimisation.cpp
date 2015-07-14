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

#include "smartOptimisation.h"

// stl library
#include <vector>
#include <algorithm>

namespace optimisation {

// Function to make the optimisation
void gielis_optimisation(const std::vector< cv::Point2f >& contour, ConfigStruct2d& config_shape, Eigen::Vector4d& mean_err, Eigen::Vector4d& std_err) {

    // Convert the data into Eigen type for further optimisation
    std::vector < Eigen::Vector2d, Eigen::aligned_allocator< Eigen::Vector2d> > Data;
    for (unsigned int contour_point_idx = 0; contour_point_idx < contour.size(); contour_point_idx++)
        Data.push_back(Eigen::Vector2d((double) contour[contour_point_idx].x, (double) contour[contour_point_idx].y));

    // Declaration of the Rational Shape
    RationalSuperShape2D RS;

    // Initilisation of the parameterers
    RS.Init(config_shape.a, config_shape.b, config_shape.n1, config_shape.n2, config_shape.n3, config_shape.p, config_shape.q, config_shape.theta_offset, config_shape.phi_offset, config_shape.x_offset, config_shape.y_offset, config_shape.z_offset);

    // Run the optimisation
    double ErrorOfFit;
    RS.Optimize8D(Data, ErrorOfFit, 1);

    // test the Error Metric function
    RS.ErrorMetric (Data, mean_err, std_err);

    // Recover the different parameters
    config_shape = ConfigStruct2d(RS.Get_a(), RS.Get_b(), RS.Get_n1(), RS.Get_n2(), RS.Get_n3(), RS.Get_p(), RS.Get_q(), RS.Get_thtoffset(), RS.Get_phioffset(), RS.Get_xoffset(), RS.Get_yoffset(), RS.Get_zoffset());

}

// Reconstruction using the Gielis formula
void gielis_reconstruction(const ConfigStruct2d& config_shape, std::vector< cv::Point2f >& gielis_contour, const int number_points) {

    // Initilisation of the output
    if(!gielis_contour.empty()) {
        gielis_contour.erase(gielis_contour.begin(), gielis_contour.end());
        gielis_contour.resize(number_points);
    }
    else
        gielis_contour.resize(number_points);

    for (int j = 0; j < number_points; j++) {

        /*----- Compute the radius ----*/
        double tmpRadius = 0;

        // Compute the radius
        double tmpIdx = ((double) j * 2.00 * M_PI) / ((double) number_points);
        double tmp_angle = config_shape.p * tmpIdx * 0.25 / config_shape.q;
        double tmpCos = cos(tmp_angle);
        double tmpSin = sin(tmp_angle);

        double tmp1 = (pow(fabs(tmpCos), config_shape.n2)) / config_shape.a;
        double tmp2 = (pow(fabs(tmpSin), config_shape.n3)) / config_shape.b;

        if ((tmp1 + tmp2) != 0) tmpRadius = pow((tmp1 + tmp2), -1.00 / config_shape.n1);
        else tmpRadius = 0.00;

        // Computation of x and y with denormalization
        gielis_contour[j].x = ( cos( tmpIdx + config_shape.theta_offset) * tmpRadius + config_shape.x_offset );
        gielis_contour[j].y = ( sin( tmpIdx + config_shape.theta_offset) * tmpRadius + config_shape.y_offset );
    }
}

}
