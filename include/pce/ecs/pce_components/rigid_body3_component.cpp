#ifndef rigid_body3_component_cpp
#define rigid_body3_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component for storing information about an entity's body
-----------------------------------------------------------------*/

#include <utility>
#include <vector>
#include <glm/vec3.hpp>


namespace pce {

using Edge3 = std::pair<glm::dvec3, glm::dvec3>;

struct RigidBody3 {
  std::vector<glm::dvec3> vertices;
  std::vector<pce::Edge3> edges;
};

}

#endif /* rigid_body3_component_cpp */
