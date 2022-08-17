#ifndef FaceOrderRenderHeadNode_cpp
#define FaceOrderRenderHeadNode_cpp

#include "../FaceOrderRenderHeadNode.hpp"


namespace pce3d {
namespace render {

FaceOrderRenderHeadNode::FaceOrderRenderHeadNode(const std::vector<glm::dvec3>& plane_points, const uint32_t id)
  : face_id_(id), plane_points_(plane_points), close_child_(NULL), far_child_(NULL)
{}


void FaceOrderRenderHeadNode::InsertWithPlaneComparison(FaceOrderRenderNode* node)
{
  const glm::dvec3 closest_plane_point = pce3d::maths::calculateClosestPointInPlaneToPoint(
                                             plane_points_[0], plane_points_[1], plane_points_[2],
                                             node->closest_corner_);
  const double plane_point_distance = pce3d::maths::calculateDistanceBetweenVectors(
                                          closest_plane_point, glm::dvec3(0, 0, 0));
  
  if (plane_point_distance <= node->closest_corner_distance_)
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

std::vector<uint32_t> FaceOrderRenderHeadNode::GetListAtHeadNode()
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

#endif /* FaceOrderRenderHeadNode_cpp */
