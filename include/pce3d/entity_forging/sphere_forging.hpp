#ifndef sphere_forging_hpp
#define sphere_forging_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
functions for creating sphere entities
-----------------------------------------------------------------*/
#include <vector>

extern ControlPanel control;

namespace pce3d {
namespace forge {

using Entity = uint32_t;

Entity forgeSphereEntity(const double radius, const glm::dvec3 location,  const std::vector<int> color);
  

}
}




#endif /* sphere_forging_hpp */
