#ifndef sheet_forging_hpp
#define sheet_forging_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for forging two-dimensional sheets
-----------------------------------------------------------------*/
#include <utility>
#include <unordered_map>
#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>
#include "../maths/functions/quaternion_functions.hpp"
#include "functions/forge_functions.hpp"
#include "base_entity_forging.hpp"

extern ControlPanel control;

namespace pce3d {
namespace forge {

using Entity = uint32_t;
using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;
using EdgeMap = std::vector<std::pair<uint32_t, uint32_t>>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;

Entity forgeRectSheetEntity(const double w, const double l, const glm::dvec3& center, 
                            const double angle, const glm::dvec3& axis_of_rotation,
                            const std::vector<int>& color);


Entity forgeTriangleSheetEntity(const std::vector<glm::dvec3>& triangle_points,
                                const std::vector<int>& color);


}
}





#endif /* sheet_forging_hpp */
