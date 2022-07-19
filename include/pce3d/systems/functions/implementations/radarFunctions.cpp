#ifndef radarFunctions_cpp
#define radarFunctions_cpp

#include <cmath>
#include "../radarFunctions.hpp"
#include "../../../maths/functions/sign.hpp"

namespace pce3d {
namespace radar {

const double LENSE_INDEX = 0.1;
const double PIXEL_ANGLE = 1.0;

const double PI = 3.14159265;

glm::dvec2 convertPointOnViewSphereToPixel(glm::dvec3 point) {

  /* if point is behind camera, render offscreen */
  if (point.z <= 0) {
    return glm::dvec2(2000, 2000);
  }

  /* calculate y pixel */ 
  const double y_point_angle = abs(atan(point.y/point.z));
  const double y_pix_angle_arc_length = abs((PIXEL_ANGLE * PI/180.0)); 
  const double y_point_arc_length = y_point_angle * (PIXEL_ANGLE * PI/180.0);
  const double y_pixel = 90.0 * y_point_arc_length/y_pix_angle_arc_length * pce::math::sign(point.y);

  /* calculate x pixel */
  const double x_point_angle = abs(atan(point.x / point.z));
  const double x_pix_angle_arc_length = abs(LENSE_INDEX * (PIXEL_ANGLE * PI/180.0));
  const double x_point_arc_length = abs(x_point_angle * LENSE_INDEX);
  const double x_pixel = x_point_arc_length/x_pix_angle_arc_length * pce::math::sign(point.x);

  return glm::dvec2(x_pixel, y_pixel);
}


}
}



#endif /* radarFunctions_cpp */
