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

namespace mathutils {

  // Maximum between three values
  inline unsigned int get_maximum(unsigned int r, unsigned int g, unsigned int b) { return (r >= g) ? ((r >= b) ? r : b) : ((g >= b) ? g : b); }

  // Minimum between three values
  inline unsigned int get_minimum(unsigned int r, unsigned int g, unsigned int b) { return (r <= g) ? ((r <= b) ? r : b) : ((g <= b) ? g : b); }
  
}

#endif /* MATH_UTILS_H_ */
