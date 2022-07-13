#ifndef PhysicsSystem_cpp
#define PhysicsSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system to handle game physics (mostly regardiing player movement)
-----------------------------------------------------------------*/

#include <ezprint.cpp>

#include "../System.cpp"
#include "physicsSystemFunctions.cpp"
#include "../../constants/static_variables.cpp"

extern ControlPanel control;

namespace pce{
class PhysicsSystem : public ISystem {
public:

  PhysicsSystem() {
    time_ = 0.0;
    previous_time_ = 0.0;
  }

  void UpdateEntities(const double sdl_time) {
    previous_time_ = time_;
    time_ = sdl_time;
    for (auto const& entity : entities) {
      auto& motion = control.GetComponent<pce::Motion>(entity);
      auto& orientation = control.GetComponent<pce::Orientation>(entity);
      if (motion.is_airborne) {
        double time_change = time_ - previous_time_;
        ezp::print_item("********* calculating airborne position *********");
        pce::phys::calculateAirbornePosition(motion, orientation, (time_ - previous_time_)); 
      }

      pce::phys::checkForMovementObstructions(orientation, motion);

       
    }
  }

private:
  double time_;
  double previous_time_;

};
}
#endif /* PhysicsSystem_cpp */
