#ifndef JoystickSystem_cpp
#define JoystickSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system to handle the player's physical controller and button 
presses.
note: this includes time.
-----------------------------------------------------------------*/

#include <ezprint.cpp>
#include <virtual_keyboard.cpp>

#include "../System.cpp"

extern ControlPanel control;

namespace pce{
class JoystickSystem : public ISystem {
public:
  
  JoystickSystem() {
    ezp::print_item("creating JoystickSystem");
  }


  void UpdateEntities(double sdl_time) {
    current_time_ = sdl_time;
    for (auto const& entity : entities) {
      auto& entity_joystick = control.GetComponent<pce::Joystick>(entity);

      
      // when add timing, add details to the transformation from the raw joystick
      // report to the player entity's joystick report (will use time for this)
      entity_joystick.keyboard_report = keyboard_.check_buttons(); 
      entity_joystick.mouse_report = mouse_.PollMouse(); 
    }
  }

private:
  double current_time_;
  double time_change_;
  VirtualKeyboard keyboard_;
  mouse::VirtualMouse mouse_;
  
};
}
#endif /* JoystickSystem_cpp */
