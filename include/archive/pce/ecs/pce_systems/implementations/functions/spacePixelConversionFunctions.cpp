#ifndef spacePixelConversionFunctions_cpp
#define spacePixelConversionFunctions_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for converting 2d pixels to 3space
-----------------------------------------------------------------*/

#include <cmath>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <ezprint.cpp>
#include <sign.cpp>
#include "../../../constants/static_variables.cpp"

namespace pce {
namespace pix_map {

const double PI = 3.14159265;


glm::dvec2 fastconvertPointOnViewSphereToPixel(const glm::dvec3& point,
                                               const glm::dvec3& view_sphere_center) {
  /* check if object is in field of view*/
  if (point.z < view_sphere_center.z) {
    const double y_circle_radius = global_const::lense_curve_index * sqrt(pow(point.y, 2.0) + pow(point.z, 2.0)); 
    const double y_point_angle = abs(atan(point.y/point.z));
    const double y_pix_angle_arc_length = y_circle_radius * abs((global_const::pixel_angle_in_3space * PI/180.0)); 
    const double y_point_arc_length = y_point_angle * (global_const::pixel_angle_in_3space * PI/180.0);
    const double y_pixel = 90.0 * y_point_arc_length/y_pix_angle_arc_length * pce::math::sign(point.y);
    // const double y_pixel = y_point_arc_length/y_pix_angle_arc_length * pce::math::sign(point.y);

    const double x_point_angle = abs(atan(point.x / point.z));
    const double x_pix_angle_arc_length = abs(global_const::lense_curve_index * global_const::view_sphere_radius * (global_const::pixel_angle_in_3space * PI/180.0));
    const double x_point_arc_length = abs(x_point_angle * global_const::lense_curve_index * global_const::view_sphere_radius);
    const double x_pixel = x_point_arc_length/x_pix_angle_arc_length * pce::math::sign(point.x);
    // const double x_pixel = 5 * x_point_arc_length/x_pix_angle_arc_length * pce::math::sign(point.x);
    return glm::dvec2(x_pixel, y_pixel);
  }

  /* if object is not in field of view, return off screen coordinates */
  return glm::dvec2(3000, 3000);
}



}
}



#endif /* spacePixelConversionFunctions_cpp */
