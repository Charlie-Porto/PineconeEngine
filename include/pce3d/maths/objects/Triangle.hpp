#ifndef Triangle_hpp
#define Triangle_hpp

#include <glm/vec2.hpp>

namespace pce3d {
namespace maths {

struct Triangle {
  glm::dvec2 A;
  glm::dvec2 B;
  glm::dvec2 C;
};

}
}

#endif /* Triangle_hpp */
