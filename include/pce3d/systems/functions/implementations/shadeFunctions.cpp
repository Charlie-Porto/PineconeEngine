#ifndef shadeFunctions_cpp
#define shadeFunctions_cpp

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



void calculateFaceBrightnessForSpherePixels(const glm::dvec3& light_direction,
                                            const glm::dvec3& sphere_center,
                                            const double sphere_radius,
                                            const PixelMap& outline_pixels,
                                            pce::PixelShadeMap& pixel_shades) {
  pixel_shades.clear();
  for (auto const& [A, B] : outline_pixels) {
    for (int i_x = int(A.x); i_x <= int(B.x); ++i_x) {
      const glm::dvec2 p = glm::dvec2(i_x, A.y);
      pixel_shades[p] = ((i_x - A.x) / (B.x - A.x));

      const glm::dvec3 viewsphere_point = glm::normalize(radar::convertPixelToPointOnViewSphere(p / 10.0));
      glm::dvec3 entity_sphere_point;

      try {
        entity_sphere_point = pce::maths::calculateClosestPointVectorIntersectsSphere(
                                                  glm::dvec3(0, 0, 0),
                                                  viewsphere_point,
                                                  sphere_center,
                                                  sphere_radius);
      } catch (double discriminant) {
        // std::cout << 'x' << '\n';
        // std::cout << "entity sphere point: " << entity_sphere_point.x << ", " << entity_sphere_point.y << ", " << entity_sphere_point.z << '\n';
      }
      const glm::dvec3 normal_vect = entity_sphere_point - sphere_center;  
      pixel_shades[p] = calculateFaceBrightness(light_direction, normal_vect);
      // std::cout << "pixel: " << p.x << ", " << p.y << ", " << "shading: " << pixel_shades.at(p) << '\n';
    }
  }
}


}
}


#endif /* shadeFunctions_cpp */
