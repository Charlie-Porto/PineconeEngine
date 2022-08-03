#ifndef triangle_functions_hpp
#define triangle_functions_hpp

#include <cmath>
#include <utility>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>

namespace pce {
namespace maths {

std::vector<uint32_t> sortVerticesByDistance(const VertexMap& vertices, 
                                             const std::vector<uint32_t>& face);
  

glm::dvec3 determineCrawlDirection(const VertexMap& vertices, 
                                   const std::vector<uint32_t>& sorted_face);




  
}
}



#endif /* triangle_functions_hpp */
