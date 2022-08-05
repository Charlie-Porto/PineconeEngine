#ifndef forge_functions_cpp
#define forge_functions_cpp

#include "../forge_functions.hpp"


namespace pce3d {
namespace forge {


void rotateVertices(VertexMap& vertices, const double angle, const glm::dvec3& axis,
                    const glm::dvec3& center_of_gravity) {
  for (auto& [id, vertex] : vertices) {
    vertex = vertex - center_of_gravity;
    vertex = pce::rotateVector3byAngleAxis(vertex, angle, axis);
    vertex = vertex + center_of_gravity;
  }
}


}
}


#endif /* forge_functions_cpp */
