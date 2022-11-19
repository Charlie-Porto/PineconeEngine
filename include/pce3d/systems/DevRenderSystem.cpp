#ifndef DevRenderSystem_cpp
#define DevRenderSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system to render specific points to assist the dev process
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>
#include "../maths/functions/quaternion_functions.hpp"
#include "../utilities/functions/glm_hash.hpp"
#include "functions/spaceMapFunctions.hpp"
#include "functions/radarFunctions.hpp"
#include "../utilities/functions/quickdraw.hpp"
#include "../utilities/functions/render_functions.hpp"

namespace pce3d {
class DevRenderSystem {
public:
  DevRenderSystem() : next_point_id_(1) {}

  void AddPointToPointColorMap(const glm::dvec3& point, const std::vector<int>& color, double radius = 1.0) 
  {
    point_map_[next_point_id_] = point;
    point_color_map_[next_point_id_] = color;
    point_radius_map_[next_point_id_] = radius;
    point_if_need_rotation_map_[next_point_id_] = false;
    ++next_point_id_;
  } 

  void AddUnRotatedPointToPointColorMap(const glm::dvec3& point, const std::vector<int>& color, double radius = 1.0) 
  {
    /* rotate point */
    point_map_[next_point_id_] = point;
    point_color_map_[next_point_id_] = color;
    point_radius_map_[next_point_id_] = radius;
    point_if_need_rotation_map_[next_point_id_] = true;
    ++next_point_id_;
  } 

  void RenderPoints(const glm::dvec3& cam_transform, const glm::dquat& cam_versor) 
  {
    // for (auto const& [point, color] : point_color_map_) 
    for (auto const& [id, point] : point_map_)
    {
      glm::dvec3 rotated_point = point;
      if (point_if_need_rotation_map_.at(id))
      {
        // std::cout << "rotating point before render" << '\n';
        rotated_point = point - cam_transform;  
        rotated_point = pce::rotateVector3byQuaternion(rotated_point, cam_versor);     
      }
      double distance = sqrt(glm::dot(rotated_point, rotated_point));
      // double distance = sqrt(glm::dot(point, point));
      // const glm::dvec3 vs_intersection = glm::normalize(rotated_point);
      const glm::dvec3 vs_intersection = glm::normalize(rotated_point);
      const glm::dvec2 pixel = radar::convertPointOnViewSphereToPixel(vs_intersection, true, false);
      pce::quickdraw::drawFilledCircleClean(pixel, point_radius_map_.at(id) * 50.0 / distance, point_color_map_.at(id));
    }
    next_point_id_ = 0;
    point_map_.clear();
    point_color_map_.clear();
    point_radius_map_.clear();
    point_if_need_rotation_map_.clear();
  }

private:
  int next_point_id_;
  std::unordered_map<uint32_t, glm::dvec3> point_map_;
  std::unordered_map<uint32_t, std::vector<int>> point_color_map_;
  std::unordered_map<uint32_t, double> point_radius_map_;
  std::unordered_map<uint32_t, double> point_if_need_rotation_map_;

};
}
#endif /* DevRenderSystem_cpp */
