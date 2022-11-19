#ifndef cylinder_forging_hpp
#define cylinder_forging_hpp

#include <vector>
#include <utility>
#include <unordered_map>

#include <glm/vec3.hpp>
#include "../functions/forge_functions.hpp" 
#include "../base_entity_forging.hpp"
#include "../../maths/functions/quaternion_functions.hpp"

#include <cmath>

extern ControlPanel control;

namespace pce3d {
namespace forge {

uint32_t forgeCylinder(
    const double h
  , const double r
  , const glm::dvec3& center
  , const std::vector<int>& color
  , const int sides
  , const double angle
  , const glm::dvec3 axis
);

}
}




#endif /* cylinder_forging_hpp */
