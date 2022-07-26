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

extern ControlPanel control;

namespace pce3d {
namespace forge {

using Entity = uint32_t;
using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;
using EdgeMap = std::vector<std::pair<uint32_t, uint32_t>>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;

Entity forgeSheetEntity(const double w, const double l, const glm::dvec3& center, 
                        const glm::dquat& local_rotation, const std::vector<int>& color);


}
}





#endif /* sheet_forging_hpp */
