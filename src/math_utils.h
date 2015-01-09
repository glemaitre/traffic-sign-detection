#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

namespace mathutils {

  // Maximum between three values
  inline unsigned int get_maximum(unsigned int r, unsigned int g, unsigned int b) { return (r >= g) ? ((r >= b) ? r : b) : ((g >= b) ? g : b); }

  // Minimum between three values
  inline unsigned int get_minimum(unsigned int r, unsigned int g, unsigned int b) { return (r <= g) ? ((r <= b) ? r : b) : ((g <= b) ? g : b); }
  
}

#endif /* MATH_UTILS_H_ */
