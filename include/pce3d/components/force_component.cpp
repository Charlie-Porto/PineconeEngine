#ifndef force_component_cpp
#define force_component_cpp

#include <unordered_map>
#include <glm/vec3.hpp>

namespace pce {

struct Force {
  double of_gravity;
  double of_contacts;
  std::unordered_map<uint32_t, glm::dvec3> contact_points;
  std::unordered_map<uint32_t, glm::dvec3> contact_directions;
  std::unordered_map<uint32_t, double> surfaces_of_contact_roughness;
  std::unordered_map<uint32_t, double> momentums;
  std::unordered_map<uint32_t, double> rotational_momentums;
  std::unordered_map<uint32_t, double> sequential_collisions_by_entity;
};

}

#endif /* force_component_cpp */
