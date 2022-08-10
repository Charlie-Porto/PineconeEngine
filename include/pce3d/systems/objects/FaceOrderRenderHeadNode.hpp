#ifndef FaceOrderRenderHeadNode_hpp
#define FaceOrderRenderHeadNode_hpp

#include <vector>
#include <glm/vec3.hpp>
#include "FaceOrderRenderNode.hpp"
#include "../../maths/functions/plane_functions.hpp"
#include "../../maths/functions/vector_functions.hpp"

namespace pce3d {
namespace render {

class FaceOrderRenderHeadNode
{
public:
  FaceOrderRenderHeadNode(const std::vector<glm::dvec3>& plane_points, const uint32_t id);
  void InsertWithPlaneComparison(FaceOrderRenderNode* node);
  std::vector<uint32_t> GetListAtHeadNode();

  uint32_t face_id_;
  std::vector<glm::dvec3> plane_points_;
private:
  FaceOrderRenderNode* close_child_;
  FaceOrderRenderNode* far_child_;
};

}
}




#endif /* FaceOrderRenderHeadNode_hpp */
