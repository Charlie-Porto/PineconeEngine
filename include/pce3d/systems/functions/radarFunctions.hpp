#ifndef radarFunctions_hpp
#define radarFunctions_hpp

#include <iostream>
#include <cmath>
#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include <glm/geometric.hpp>
#include "../../maths/functions/sign.hpp"

namespace pce3d {
namespace radar {


glm::dvec2 convertPointOnViewSphereToPixel(const glm::dvec3& point, bool is_center_of_gravity, bool is_first_pass);


glm::dvec3 convertPixelToPointOnViewSphere(const glm::dvec2& pixel);


glm::dvec3 interpolateViewSphereIntersectionPoint(const glm::dvec3& point, const double z_target,
                                                  const glm::dvec3& connected_on_screen_point);

}
}



#endif /* radarFunctions_hpp */
