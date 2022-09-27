#ifndef SphereFactory_hpp
#define SphereFactory_hpp


#include <cmath>
#include <algorithm>
#include <vector>
#include <glm/vec3.hpp>
#include "../sphere_forging.hpp"
#include "../../maths/functions/pce_psuedo_randomness.hpp"
  


namespace pce3d {
namespace efactory {

class SphereFactory {
public:
  SphereFactory(
      double radius_max = 1
    , glm::dvec3 factory_location = glm::dvec3(0, 40, 20)
    , std::vector<int> color = {200, 100, 150, 255}
    , glm::dvec3 velocity = glm::dvec3(0, 0, 0)
    // , double velocity_component_variance = 3
    , double gravitational_force = 1.0
    , double bounciness = 0.7
  );

  void CreateSphereParticle();
    
private:
  double radius_max_;
  glm::dvec3 factory_location_;
  std::vector<int> color_;
  glm::dvec3 velocity_;
  // double velocity_component_variance_;
  double gravitational_force_;
  double bounciness_;
};

}
}



#endif /* SphereFactory_hpp */
