#ifndef radar_component_cpp
#define radar_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component that stores information about where an entity will be 
rendered based on:
  - its coordinates in 3space 
  - the camera's position
  - the camera's angle
-----------------------------------------------------------------*/

#include <unordered_map>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace pce {

struct Radar {
  glm::dvec3 view_surface_intersection_point;
  glm::dvec2 intersection_point_corresponding_pixel; 
  double distance_from_camera;
  std::unordered_map<uint32_t, glm::dvec2> vertices_corresponding_pixels;
};

}

#endif /* radar_component_cpp */
