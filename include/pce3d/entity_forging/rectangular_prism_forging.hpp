#ifndef rectangular_prism_forging_hpp
#define rectangular_prism_forging_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to forge rectangular prism entities
-----------------------------------------------------------------*/

#include <vector>
#include <utility>
#include <unordered_map>
#include <glm/vec3.hpp>
#include <glm/ext/quaternion_double.hpp>
#include "functions/forge_functions.hpp"
#include "base_entity_forging.hpp"

extern ControlPanel control;

namespace pce3d {
namespace forge {

Entity forgeRectPrismEntity(
    const double w
  , const double h
  , const double l
  , const glm::dvec3& center
  , const double angle
  , const glm::dvec3& axis
  , const std::vector<int>& color
  , bool is_livebod = false
  , const double g_force = 0.0
  , const glm::dvec3& velocity = glm::dvec3(0, 0, 0)
  , const glm::dvec3& axis_of_rotation = glm::dvec3(1, 0, 0)
  , const double rotation_speed = 0.0);


VertexMap calculateRectPrismOriginalVertexLocations(const double w, const double h, const double l, 
                                                    const glm::dvec3& center);
                          

FaceVertexMap assignVerticesToFaces();

EdgeMap assignEdgesToVertices();

VertexVertexMap assignVerticestoVertices();

void registerRectPrismEntityWithOrderRenderSystem(double size);




}
}




#endif /* rectangular_prism_forging_hpp */
