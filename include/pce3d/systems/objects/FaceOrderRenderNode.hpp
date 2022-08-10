#ifndef FaceOrderRenderNode_hpp
#define FaceOrderRenderNode_hpp


#include <glm/vec3.hpp>
#include "../../maths/functions/vector_functions.hpp"

namespace pce3d {
namespace render {

class FaceOrderRenderNode
{
public:
  FaceOrderRenderNode(const glm::dvec3& closest_corner, const uint32_t id);
  void InsertWithCornerComparison(FaceOrderRenderNode* node);
  std::vector<uint32_t> GetListAtNode();

  uint32_t face_id_;
  glm::dvec3 closest_corner_;
  double closest_corner_distance_;
private:
  FaceOrderRenderNode* close_child_;
  FaceOrderRenderNode* far_child_;
};

}
}




#endif /* FaceOrderRenderNode_hpp */
