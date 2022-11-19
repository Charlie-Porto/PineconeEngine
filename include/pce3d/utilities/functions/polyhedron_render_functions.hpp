#ifndef polyhedron_render_functions_hpp
#define polyhedron_render_functions_hpp

#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include <vector>
#include "../../maths/functions/vector_functions.hpp"
#include "../../systems/functions/radarFunctions.hpp"
#include "SDL_cartesian_conversion.hpp"

#include "render_functions.hpp"


namespace pce3d {
namespace render {

void renderTransparentCylinder(
    const glm::dvec3& A
  , const glm::dvec3& B
  , const double radius
  , const std::vector<int> color
);


}
}




#endif /* polyhedron_render_functions_hpp */
