#ifndef orderForRenderFunctions_hpp
#define orderForRenderFunctions_hpp

#include <unordered_map>
#include <utility>
#include <vector>
#include "../../maths/functions/plane_functions.hpp"
#include "../objects/orderTag.hpp"
#include "../../maths/functions/vertex_functions.hpp"

extern ControlPanel control;

namespace pce3d {
namespace render_order {

void insertEntityIntoRenderOrderVector(const std::pair<uint32_t, double>& entity, 
                                       std::vector<std::pair<uint32_t, double>>& order_vector);
                                       
void insertEntityIntoRenderOrderVector(const std::pair<uint32_t, double>& entity, 
                                       std::vector<std::pair<uint32_t, double>>& order_vector);

void insertEntityIntoRenderOrderVectorLinear(const std::pair<uint32_t, double>& entity, 
                                             std::vector<std::pair<uint32_t, double>>& order_vector);

std::pair<bool, size_t> tryInsertEntityIntoRenderOrderMap(const orderTag& entity_tag, std::vector<orderTag>& order_list);

void insertEntityBetweenVerticesIntoRenderOrderMapAtIndex(const orderTag& entity_tag, size_t i, 
                                                          const glm::dvec3 closest_vertex_rotated_pos,
                                                          std::vector<orderTag>& order_list);


}
}



#endif /* orderForRenderFunctions_hpp */
