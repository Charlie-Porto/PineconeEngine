#ifndef rigid_body2_component_cpp
#define rigid_body2_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component for storing information about an entity's body
-----------------------------------------------------------------*/

#include <utility>
#include <vector>
#include <glm/vec2.hpp>


namespace pce {

using Edge2 = std::pair<glm::dvec2, glm::dvec2>;

struct RigidBody2 {
  std::vector<glm::dvec2> vertices;
  std::vector<pce::Edge2> edges;
};

}

#endif /* rigid_body2_component_cpp */
