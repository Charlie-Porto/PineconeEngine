#ifndef orderForRenderFunctions_hpp
#define orderForRenderFunctions_hpp

#include <utility>
#include <vector>

namespace pce3d {
namespace render_order {

void insertEntityIntoRenderOrderVector(const std::pair<uint32_t, double>& entity, 
                                       std::vector<std::pair<uint32_t, double>>& order_vector);

void insertEntityIntoRenderOrderVectorLinear(const std::pair<uint32_t, double>& entity, 
                                             std::vector<std::pair<uint32_t, double>>& order_vector);
}
}



#endif /* orderForRenderFunctions_hpp */
