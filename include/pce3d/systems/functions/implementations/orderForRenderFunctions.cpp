#ifndef orderForRenderFunctions_cpp
#define orderForRenderFunctions_cpp

#include "../orderForRenderFunctions.hpp"
#include <ezprint.cpp>

namespace pce3d {
namespace render_order {

void insertEntityIntoRenderOrderVector(const std::pair<uint32_t, double>& entity, 
                                       std::vector<std::pair<uint32_t, double>>& order_vector) {

  size_t left = 0;
  size_t right = order_vector.size();
  size_t index = 0;

  while (left < right) {
    size_t mid = (left + right) / 2;
    bool condition_index_1 = false;
    bool condition_index_2 = false;

    /* check index #1 */
    if (mid <= 0) { condition_index_1 = true; }
    else if (order_vector[mid+1].second < entity.second) { condition_index_1 = true; }

    /* check index #2 */
    if (mid >= order_vector.size()) { condition_index_2 = true; }
    if (order_vector[mid].second > entity.second) { condition_index_2 = true; }

    /* check indices and adjust left or right if needed */
    if (condition_index_1 && condition_index_2) { index = mid; break; }
    else if (!condition_index_1) { left = mid; }
    else if (!condition_index_2) { right = mid; }
  }
  if (order_vector.size() > 0) { order_vector.insert(order_vector.begin() + index+1, entity); } 
  else { order_vector.push_back(entity); }
}

}
}




#endif /* orderForRenderFunctions_cpp */
