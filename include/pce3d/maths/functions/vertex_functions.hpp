#ifndef vertex_functions_hpp
#define vertex_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to relating to polyhedron vertices
-----------------------------------------------------------------*/

#include <algorithm>
#include <unordered_map>
#include <glm/vec3.hpp>
#include "vector_functions.hpp"

namespace pce3d {
namespace maths {

using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;

uint32_t calculateClosestVertexToPoint(const glm::dvec3& point, const VertexMap& vertices);

}
}





#endif /* vertex_functions_hpp */
