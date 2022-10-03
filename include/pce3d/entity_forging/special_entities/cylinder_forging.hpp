#ifndef cylinder_forging_hpp
#define cylinder_forging_hpp

#include <glm/vec3.hpp>
#include "../functions/forge_functions.hpp" 
#include "../base_entity_forging.hpp"

extern ControlPanel control;

namespace pce3d {
namespace forge {

uint32_t forgeCylinder(
    const double h
  , const double r
  , const glm::dvec3& center
  , const std::vector<int>& color
);

}
}




#endif /* cylinder_forging_hpp */
