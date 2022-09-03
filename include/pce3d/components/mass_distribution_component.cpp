#ifndef mass_distribution_component_cpp
#define mass_distribution_component_cpp

#include <unordered_map>
#include <glm/vec3.hpp>

namespace pce {

struct MassDistribution 
{
  std::unordered_map<uint32_t, glm::dvec3> mass_zones;
  std::unordered_map<glm::dvec3, uint32_t> coordinates_to_zone_map;
  std::unordered_map<uint32_t, glm::dvec3> mass_zones_rotational_velocities;
  std::unordered_map<uint32_t, double> mass_zones_distance_from_center_of_mass;
  double zone_mass;
  uint32_t outermost_zone;
  bool need_to_recalculate_velocities;
};

}

#endif /* mass_distribution_component_cpp */
