#ifndef massMapFunctions_hpp
#define massMapFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to assist the massMapSystem
-----------------------------------------------------------------*/

#include <vector>

#include <glm/vec3.hpp>

#include "../../pce3d.hpp"
#include "spaceMapFunctions.hpp"
#include "../../maths/functions/vector_functions.hpp"
#include "../../maths/functions/rounding_functions.hpp"


namespace pce3d {
namespace massmap {

using VertexMap = std::unordered_map<uint32_t, glm::dvec3>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;

std::vector<glm::dvec3> findRectFaceMassZones(
    const uint32_t face_id
  , const FaceVertexMap& faces
  , const VertexMap& vertices
);

std::vector<glm::dvec3> findTriFaceMassZones(
    const uint32_t face_id
  , const FaceVertexMap& faces
  , const VertexMap& vertices
);

std::vector<glm::dvec3> findFaceMassZones(
    const uint32_t face_id
  , const FaceVertexMap& faces
  , const VertexMap& vertices
);

std::vector<glm::dvec3> findAllSurfaceMassZones(
    const FaceVertexMap& faces
  , const VertexMap& vertices
);

void findAllMassZonesGivenSurfaceZones(
    const glm::dvec3& center_of_mass
  , const std::vector<glm::dvec3>& face_mass_zones
  , std::unordered_map<uint32_t, glm::dvec3>& mass_zone_map 
  , std::unordered_map<glm::dvec3, uint32_t>& coordinate_zone_map 
  , std::unordered_map<uint32_t, double>& mass_zone_distance_map 
);



}
}




#endif /* massMapFunctions_hpp */
