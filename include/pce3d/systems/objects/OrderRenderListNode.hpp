#ifndef OrderRenderListNode_hpp
#define OrderRenderListNode_hpp


#include <vector>
#include "orderTag.hpp"

extern uint32_t pce3d::render_order::getCloserOfTwoEntitiesToOrigin(const pce3d::orderTag& a_entity_tag, const pce3d::orderTag& b_entity_tag);

namespace pce3d {
namespace render_order {

class OrderRenderListNode
{
public:
  OrderRenderListNode(const orderTag& tag);
  // OrderRenderListNode(const orderTag& tag) : close_child_(NULL), far_child_(NULL), order_tag_(tag);
  void InsertNodeInTree(OrderRenderListNode* node);
  std::vector<uint32_t> GetListAtNode();

  orderTag order_tag_;
private:
  OrderRenderListNode* close_child_;
  OrderRenderListNode* far_child_;


};

}
}




#endif /* OrderRenderListNode_hpp */
