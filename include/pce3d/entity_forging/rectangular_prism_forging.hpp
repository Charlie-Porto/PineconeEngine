#ifndef rectangular_prism_forging_hpp
#define rectangular_prism_forging_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to forge rectangular prism entities
-----------------------------------------------------------------*/

#include <vector>
#include <unordered_map>
#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>

extern ControlPanel control;

namespace pce3d {
namespace forge {

using Entity = uint32_t;
using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;
using FaceNormalVectorMap = std::unordered_map<uint32_t, glm::dvec3>;
using FaceCenterPointMap = std::unordered_map<uint32_t, glm::dvec3>;


Entity forgeRectPrismEntity(const double w, const double h, const double l,
                            const glm::dvec3& center, const glm::dquat& local_rotation,
                            const std::vector<int>& color);


VertexMap calculateRectPrismOriginalVertexLocations(const double w, const double h, const double l, 
                                                    const glm::dvec3& center);
                          

FaceVertexMap assignVerticesToFaces();







}
}




#endif /* rectangular_prism_forging_hpp */
