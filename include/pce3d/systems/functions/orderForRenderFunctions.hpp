#ifndef orderForRenderFunctions_hpp
#define orderForRenderFunctions_hpp

#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>
#include "../../maths/functions/plane_functions.hpp"
#include "../objects/orderTag.hpp"
#include "../../maths/functions/quaternion_functions.hpp"
#include "../DevRenderSystem.cpp"


extern ControlPanel control;
extern pce3d::DevRenderSystem dev_render_system;

namespace pce3d {
namespace render_order {

// void insertEntityIntoRenderOrderVector(const std::pair<uint32_t, double>& entity, 
//                                        std::vector<std::pair<uint32_t, double>>& order_vector);
                                       
// void insertEntityIntoRenderOrderVector(const std::pair<uint32_t, double>& entity, 
//                                        std::vector<std::pair<uint32_t, double>>& order_vector);

// void insertEntityIntoRenderOrderVectorLinear(const std::pair<uint32_t, double>& entity, 
//                                              std::vector<std::pair<uint32_t, double>>& order_vector);

// std::pair<bool, size_t> tryInsertEntityIntoRenderOrderMap(const orderTag& entity_tag, std::vector<orderTag>& order_list);

// void insertEntityBetweenVerticesIntoRenderOrderMapAtIndex(const orderTag& entity_tag, size_t i, 
//                                                           const glm::dvec3 closest_vertex_rotated_pos,
//                                                           std::vector<orderTag>& order_list);

// void insertEntityIntoOrderMap(const orderTag& entity_tag,
//                               std::vector<orderTag>& order_list, size_t start_position);

// void insertEntityIntoOrderMapBesideIndex(const orderTag& entity_tag, size_t i,
//                                          std::vector<orderTag>& order_list);

uint32_t getCloserOfTwoOverlappingEntitiesToOrigin(const orderTag& a_entity_tag, 
                                                   const orderTag& b_entity_tag);

uint32_t getCloserOfTwoEntitiesToOrigin(const orderTag& a_entity_tag, const orderTag& b_entity_tag);


void insertEntityIntoOrderMapBinary(const orderTag& entity_tag,
                                    std::vector<orderTag>& order_list);


void insertEntityIntoOrderMapStartingAtEnd(const orderTag& entity_tag,
                                           std::vector<orderTag>& order_list);

}
}



#endif /* orderForRenderFunctions_hpp */
