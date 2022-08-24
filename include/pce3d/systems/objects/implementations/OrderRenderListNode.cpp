#ifndef OrderRenderListNode_cpp
#define OrderRenderListNode_cpp

#include "../OrderRenderListNode.hpp"


namespace pce3d {
namespace render_order {

  OrderRenderListNode::OrderRenderListNode(const orderTag& tag) : order_tag_{tag}, close_child_{NULL}, far_child_{NULL} {}

  void OrderRenderListNode::InsertNodeInTree(OrderRenderListNode* node)
  {
    uint32_t closer_entity
        = pce3d::render_order::getCloserOfTwoEntitiesToOrigin(order_tag_, node->order_tag_);
    
    // std::cout << "closer entity: " << closer_entity << '\n';
    if (closer_entity == order_tag_.entity)
    {
      if (far_child_ == NULL) { far_child_ = node; }
      else { far_child_->InsertNodeInTree(node); }
    }
    else 
    {
      if (close_child_ == NULL) { close_child_ = node; }
      else { close_child_->InsertNodeInTree(node); }
    }
  }


  std::vector<uint32_t> OrderRenderListNode::GetListAtNode()
  { 
    std::vector<uint32_t> list = {order_tag_.entity};
    if (close_child_ != NULL)
    {
      std::vector<uint32_t> close_child_list = close_child_->GetListAtNode();
      list.insert(list.end(), close_child_list.begin(), close_child_list.end());
      delete close_child_;
    }
    if (far_child_ != NULL)
    {
      std::vector<uint32_t> far_child_list = far_child_->GetListAtNode();
      far_child_list.insert(far_child_list.end(), list.begin(), list.end());
      delete far_child_;
      return far_child_list;
    }
    return list;
  }


}
}





#endif /* OrderRenderListNode_cpp */
