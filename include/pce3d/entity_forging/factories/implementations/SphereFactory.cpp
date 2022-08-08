#ifndef SphereFactory_cpp
#define SphereFactory_cpp

#include "../SphereFactory.hpp"


namespace pce3d {
namespace efactory {

SphereFactory::SphereFactory(
    double radius_max
  , double radius_min
  , glm::dvec3 factory_location
  , double x_variance
  , double y_variance
  , double z_variance
  , glm::dvec3 velocity
  , double velocity_component_variance
  , double gravitational_force
  , double gravitational_force_variance
)
  :   radius_max_{radius_max}
    , radius_min_{radius_min} 
    , factory_location_{factory_location}
    , x_variance_{x_variance}
    , y_variance_{y_variance}
    , z_variance_{z_variance}
    , velocity_{velocity}
    , velocity_component_variance_{velocity_component_variance}
    , gravitational_force_{gravitational_force}
    , gravitational_force_variance_{gravitational_force_variance}
{
  std::cout << "sphere factory created" << '\n';

}
  

void SphereFactory::CreateSphereParticle() {
  pce3d::forge::forgeSphereEntity(
      radius_max_
    , factory_location_
    , {13, 170, 100, 255}
    , velocity_
    , gravitational_force_);
}
    



}
}





#endif /* SphereFactory_cpp */
