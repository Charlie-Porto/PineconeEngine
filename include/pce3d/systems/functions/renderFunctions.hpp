#ifndef renderFunctions_hpp
#define renderFunctions_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions to assist with rendering
-----------------------------------------------------------------*/

#include <utility>
#include <vector>
#include <unordered_map>

namespace pce3d {
namespace render {

using VertexDistanceMap = std::unordered_map<uint32_t, double>;
using FaceVertexMap = std::unordered_map<uint32_t, std::vector<uint32_t>>;

std::vector<std::pair<uint32_t, double>> orderFacesByCameraProximity(const FaceVertexMap& face_vertex_map,
                                                  const VertexDistanceMap& vertex_distance_map);

}
}





#endif /* renderFunctions_hpp */
