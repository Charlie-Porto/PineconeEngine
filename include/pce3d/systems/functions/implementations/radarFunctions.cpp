#ifndef radarFunctions_cpp
#define radarFunctions_cpp

#include "../radarFunctions.hpp"

namespace pce3d {
namespace radar {

const double LENSE_INDEX = 0.1;
const double PIXEL_ANGLE = 1.0;
const double Y_PIXEL_STRETCH = 90.0;
const double SCREEN_X = 1000.0;

const double PI = 3.14159265;

glm::dvec2 convertPointOnViewSphereToPixel(const glm::dvec3& point, bool is_center_of_gravity, bool is_first_pass) {
  glm::dvec3 mpoint = point;
  bool to_interpolate = false;
  if (!is_first_pass) {
    if (point.z < -0.2) {
      if (is_center_of_gravity) 
      {
        return glm::dvec2(0, 0); 
      }
      to_interpolate = true;
    }
    if (abs(atan(point.x/point.z) / PI * 180.0) > 70.0) {
      if (is_center_of_gravity) 
      {
        return glm::dvec2(0, 0); 
      }
      // to_interpolate = true;
    }
    if (abs(atan(point.y/point.z) / PI * 180.0) > 50.0) {
      if (is_center_of_gravity) 
      {
        return glm::dvec2(0, 0); 
      }
      // to_interpolate = true;
    }
    if (to_interpolate)
    {
      mpoint = interpolateViewSphereIntersectionPoint(point, 1.0, glm::dvec3(0, 10.0, 10.0));
    }
  }
  const double y_pixel = Y_PIXEL_STRETCH * atan(mpoint.y / mpoint.z);
  const double x_pixel = atan(mpoint.x / mpoint.z) / (PIXEL_ANGLE * PI / 180.0);
  return glm::dvec2(x_pixel, y_pixel) * pce::math::sign(mpoint.z);
}


glm::dvec3 convertPixelToPointOnViewSphere(const glm::dvec2& pixel) {
  const double z = 1.0 / (1.0 + tan(double(pixel.y) / Y_PIXEL_STRETCH) - tan(double(pixel.x) * (PIXEL_ANGLE * PI / 180.0)));
  const double x = z * tan(pixel.x * (PIXEL_ANGLE * PI / 180.0));
  const double y = z * tan(pixel.y / Y_PIXEL_STRETCH);
  // std::cout << "viewsphere point: " << x << ", " << y << ", " << z << "\n";
  return glm::dvec3(x, y, z);
}



glm::dvec3 interpolateViewSphereIntersectionPoint(const glm::dvec3& point, const double z_target,
                                                  const glm::dvec3& connected_on_screen_point) {
  std::cout << "interpolating!!!!!!!!" << '\n';
  if (point.z == connected_on_screen_point.z && point.y == connected_on_screen_point.y) {
    return (glm::normalize(glm::dvec3(point.x, connected_on_screen_point.y, z_target)));
  }
  if (point.z == connected_on_screen_point.z && point.x == connected_on_screen_point.x) {
    return (glm::normalize(glm::dvec3(connected_on_screen_point.x, point.y, z_target)));
  }
  // const glm::dvec3 direction = glm::normalize(connected_on_screen_point - point);
  const glm::dvec3 direction = glm::normalize(point - connected_on_screen_point);
  std::cout << "point: "<<point.x << ", "<< point.y << ", "<< point.z << '\n';
  std::cout << "screenpoint: "
            << connected_on_screen_point.x << ", "
            << connected_on_screen_point.y << ", "
            << connected_on_screen_point.z << '\n';
  double t = (z_target - point.z) / direction.z;
  // glm::dvec3 psuedo_intersection_point = glm::dvec3(-(point.x + t * direction.x), -(point.y + t * direction.y), z_target);
  glm::dvec3 psuedo_intersection_point = glm::dvec3((point.x + t * direction.x), -(point.y + t * direction.y), z_target);
  // glm::dvec3 psuedo_intersection_point = glm::dvec3((point.x + t * direction.x), (point.y + t * direction.y), z_target);
  glm::dvec3 unit_psuedo_int_point = glm::normalize(psuedo_intersection_point);
  std::cout << "unit_psuedo_int_point: "
            << unit_psuedo_int_point.x << ", "
            << unit_psuedo_int_point.y << ", "
            << unit_psuedo_int_point.z << '\n';

  // const double x = point.x + t * direction.x;
  // double y = point.y + t * direction.y;
  // const double z = z_target;
  // if (x < 1.0) {
    // y = sqrt(1.0 - pow(x, 2.0) * pow(z, 2.0));
    // return glm::dvec3(x, y, z);
  // } else {
    // return unit_psuedo_int_point;
  // }
  return unit_psuedo_int_point;
}




}
}



#endif /* radarFunctions_cpp */
