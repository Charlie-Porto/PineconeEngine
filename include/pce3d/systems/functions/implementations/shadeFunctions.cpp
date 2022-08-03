#ifndef shadeFunctions_cpp
#define shadeFunctions_cpp

#include "../shadeFunctions.hpp"

namespace pce3d {
namespace shade {

const double PI = 3.14159265;

double calculateFaceBrightness(const glm::dvec3& light_direction, const glm::dvec3& plane_normal_vec) {
/* returns a double between 0 and 1 */
/* 0 = black */
/* 1 = pure natural color */

  double angle_light_hits_face = acos(glm::dot(light_direction, plane_normal_vec) 
                                    / (sqrt(glm::dot(light_direction, light_direction))
                                       * sqrt(glm::dot(plane_normal_vec, plane_normal_vec))));
  
  // std::cout << "angle light hits face: " << (angle_light_hits_face / PI * 180.0) << '\n';

  return (angle_light_hits_face/PI);
}



void calculateFaceBrightnessForSpherePixels(const glm::dvec3& light_direction,
                                            const glm::dvec3& sphere_center,
                                            const double sphere_radius,
                                            const PixelMap& outline_pixels,
                                            pce::PixelShadeMap& pixel_shades) {
  pixel_shades.clear();
  for (auto const& [A, B] : outline_pixels) {
    for (double i_x = A.x; i_x <= B.x; ++i_x) {
      const glm::dvec2 p = glm::dvec2(i_x, A.y);

      const glm::dvec3 viewsphere_point = glm::normalize(radar::convertPixelToPointOnViewSphere(p / pce3d::Core3D::ORDINARY_ZOOM_INDEX_));
      glm::dvec3 entity_sphere_point;

      try {
        entity_sphere_point = pce::maths::calculateClosestPointVectorIntersectsSphere(
                                                  glm::dvec3(0, 0, 0),
                                                  viewsphere_point,
                                                  sphere_center,
                                                  sphere_radius);
      } catch (double discriminant) {}
      const glm::dvec3 normal_vect = entity_sphere_point - sphere_center;  
      pixel_shades[p] = calculateFaceBrightness(light_direction, normal_vect);
      // std::cout << "pixel: " << p.x << ", " << p.y << ", " << "shading: " << pixel_shades.at(p) << '\n';
    }
  }
}


}
}


#endif /* shadeFunctions_cpp */
