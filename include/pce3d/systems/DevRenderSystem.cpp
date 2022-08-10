#ifndef DevRenderSystem_cpp
#define DevRenderSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system to render specific points to assist the dev process
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>
#include "../maths/functions/quaternion_functions.hpp"
#include <glm_hash.hpp>
#include "functions/spaceMapFunctions.hpp"
#include "functions/radarFunctions.hpp"
#include "../utilities/functions/quickdraw.hpp"

namespace pce3d {
class DevRenderSystem {
public:

void AddPointToPointColorMap(const glm::dvec3& point, const std::vector<int>& color, double radius = 1.0) {
  point_color_map_[point] = color;
  point_radius_map_[point] = radius;
} 


void RenderPoints(const glm::dvec3& cam_transform, const glm::dquat& cam_versor) {
  for (auto const& [point, color] : point_color_map_) {
    // glm::dvec3 rotated_point = point - cam_transform;
    // double distance = sqrt(glm::dot(rotated_point, rotated_point));
    double distance = sqrt(glm::dot(point, point));
    // rotated_point = pce::rotateVector3byQuaternion(rotated_point, cam_versor);     
    // const glm::dvec3 vs_intersection = glm::normalize(rotated_point);
    const glm::dvec3 vs_intersection = glm::normalize(point);
    const glm::dvec2 pixel = radar::convertPointOnViewSphereToPixel(vs_intersection, true, true);
    pce::quickdraw::drawFilledCircleClean(pixel, point_radius_map_.at(point) * 50.0 / distance, color);
  }
  point_color_map_.clear();
  point_radius_map_.clear();
}

private:
  std::unordered_map<glm::dvec3, std::vector<int>> point_color_map_;
  std::unordered_map<glm::dvec3, double> point_radius_map_;

};
}
#endif /* DevRenderSystem_cpp */
