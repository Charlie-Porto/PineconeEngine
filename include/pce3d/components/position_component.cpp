#ifndef position_component_cpp
#define position_component_cpp

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>

namespace pce {

struct Position {
  glm::dvec3 actual_center_of_mass;
  glm::dvec3 center_of_mass_relative_to_camera;
  glm::dvec2 center_of_mass_radar_pixel;
  double distance_from_camera;
};

}

#endif /* position_component_cpp */
