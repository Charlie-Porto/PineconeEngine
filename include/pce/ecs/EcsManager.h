#ifndef EcsManager_h
#define EcsManager_h

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
class to manage the entity component system
-----------------------------------------------------------------*/
#include "ecs_implementation/ControlPanel.cpp"


/* pce components */
#include "pce_components/entity_type_component.cpp"
#include "pce_components/position3_component.cpp"
#include "pce_components/position2_component.cpp"
#include "pce_components/motion3_component.cpp"
#include "pce_components/motion2_component.cpp"
#include "pce_components/local_rotation3_component.cpp"
#include "pce_components/local_rotation2_component.cpp"
#include "pce_components/surface_component.cpp"
#include "pce_components/radar_component.cpp"
#include "pce_components/joystick_component.cpp"
#include "pce_components/rigid_body3_component.cpp"
#include "pce_components/rigid_body2_component.cpp"

extern ControlPanel control;

namespace pce {
class EcsManager {
public:
  EcsManager();



private:
 
};

}







#endif /* EcsManager_h */
