#ifndef FaceOrderRenderNode_cpp
#define FaceOrderRenderNode_cpp

#include "../FaceOrderRenderNode.hpp"

namespace pce3d {
namespace render {

FaceOrderRenderNode::FaceOrderRenderNode(const glm::dvec3& closest_corner, const uint32_t id) 
  : closest_corner_(closest_corner), face_id_(id), close_child_(NULL), far_child_(NULL)
{
  closest_corner_distance_ = pce3d::maths::calculateDistanceBetweenVectors(closest_corner, glm::dvec3(0, 0, 0));
}
   

void FaceOrderRenderNode::InsertWithCornerComparison(FaceOrderRenderNode* node)
{
  if (closest_corner_distance_ <= node->closest_corner_distance_)
  {
    if (far_child_ == NULL)
    {
      far_child_ = node;
    }
    else
    {
      far_child_->InsertWithCornerComparison(node);
    }
  }
  else
  {
    if (close_child_ == NULL)
    {
      close_child_ = node;
    }
    else
    {
      close_child_->InsertWithCornerComparison(node);
    } 
  }
}


std::vector<uint32_t> FaceOrderRenderNode::GetListAtNode()
{
  std::vector<uint32_t> list = {face_id_};
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



#endif /* FaceOrderRenderNode_cpp */
