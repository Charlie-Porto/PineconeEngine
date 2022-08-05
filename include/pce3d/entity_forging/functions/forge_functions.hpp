#ifndef forge_functions_hpp
#define forge_functions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to assist the forge process
-----------------------------------------------------------------*/

#include <unordered_map>
#include <glm/vec3.hpp>
#include "../../maths/functions/quaternion_functions.hpp"

namespace pce3d {
namespace forge {

using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;

void rotateVertices(VertexMap& vertices, const double angle, const glm::dvec3& axis,
                    const glm::dvec3& center_of_gravity);


}
}




#endif /* forge_functions_hpp */
