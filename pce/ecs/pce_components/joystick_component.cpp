#ifndef joystick_component_cpp
#define joystick_component_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
component for storing the state of keyboard and mouse inputs
-----------------------------------------------------------------*/

#include "../../tools/virtual_keyboard.hpp"
#include "../../tools/mouse_controller.hpp"

namespace pce {

struct Joystick {
  JoystickReport keyboard_report; 
  MouseReport mouse_report;
};

}

#endif /* joystick_component_cpp */
