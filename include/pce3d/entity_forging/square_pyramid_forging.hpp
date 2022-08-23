#ifndef square_pyramid_forging_hpp
#define square_pyramid_forging_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for forging square pyramids
-----------------------------------------------------------------*/

#include <cmath>
#include <vector>
#include <utility>
#include <unordered_map>
#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>
#include "base_entity_forging.hpp"

extern ControlPanel control;


namespace pce3d {
namespace forge {



Entity forgeSquarePyramidEntity(const double h, const double base_side_length, 
                                const glm::dvec3& center, const glm::dquat& local_rotation,
                                const std::vector<int>& color);



}
}





#endif /* square_pyramid_forging_hpp */
