#ifndef shadeFunctions_cpp
#define shadeFunctions_cpp

#include <iostream>
#include <cmath>
#include <glm/geometric.hpp>
#include "../shadeFunctions.hpp"


namespace pce3d {
namespace shade {

const double PI = 3.14159265;

double calculateFaceBrightness(const glm::dvec3& light_direction, const glm::dvec3& plane_normal_vec) {
/* returns a double between 0 and 2 */
/* 0 = black */
/* 1 = pure natural color */
/* 2 = white */

  double angle_light_hits_face = acos(glm::dot(light_direction, plane_normal_vec) 
                                    / (sqrt(glm::dot(light_direction, light_direction))
                                       * sqrt(glm::dot(plane_normal_vec, plane_normal_vec))));
  
  // std::cout << "angle light hits face: " << (angle_light_hits_face / PI * 180.0) << '\n';

  return (angle_light_hits_face/PI);
}



}
}


#endif /* shadeFunctions_cpp */
