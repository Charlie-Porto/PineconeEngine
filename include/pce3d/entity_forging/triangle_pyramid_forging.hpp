#ifndef triangle_pyramid_forging_hpp
#define triangle_pyramid_forging_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to forge triangle pyramids
-----------------------------------------------------------------*/

#include <cmath>
#include <vector>
#include <utility>
#include <unordered_map>
#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>

extern ControlPanel control;

namespace pce3d {
namespace forge {

using Entity = uint32_t;
using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;
using VertexVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using EdgeMap = std::vector<std::pair<uint32_t, uint32_t>>;

const double PI = 3.14159265;


Entity forgeTrianglePyramidEntity(const double h, const double base_side_length, 
                                  const glm::dvec3& center, const glm::dquat& local_rotation,
                                  const std::vector<int>& color);


}
}




#endif /* triangle_pyramid_forging_hpp */
