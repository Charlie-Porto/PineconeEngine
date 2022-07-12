#ifndef EcsManager_h
#define EcsManager_h

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
class to manage the entity component system
-----------------------------------------------------------------*/
#include "ecs/ecs_implementation/ControlPanel.cpp"

#include "ecs/pce_components/entity_type_component.cpp"
#include "ecs/pce_components/position3_component.cpp"
#include "ecs/pce_components/position2_component.cpp"
#include "ecs/pce_components/motion3_component.cpp"
#include "ecs/pce_components/motion2_component.cpp"
#include "ecs/pce_components/local_rotation3_component.cpp"
#include "ecs/pce_components/local_rotation2_component.cpp"
#include "ecs/pce_components/surface_component.cpp"
#include "ecs/pce_components/radar_component.cpp"
#include "ecs/pce_components/joystick_component.cpp"

extern ControlPanel control;

namespace pce {
class EcsManager {
public:
  EcsManager();



private:
 
};

}







#endif /* EcsManager_h */
