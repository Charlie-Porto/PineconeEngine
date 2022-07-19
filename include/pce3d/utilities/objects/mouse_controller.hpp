#ifndef mouse_controller_hpp
#define mouse_controller_hpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
struct & class to track the mouse
-----------------------------------------------------------------*/

#include "../../../pceSDL/core/Simulation.hpp"
#include "mouse_report.hpp"


namespace pce {
namespace mouse {


class VirtualMouse {
public:


  VirtualMouse() {
    mouse_report_.x_position = 0.0;
    mouse_report_.y_position = 0.0;
    mouse_report_.left_clicked = false;
    mouse_report_.right_clicked = false;
  }

  MouseReport& PollMouse() {
    prev_mouse_x = mouse_x;
    prev_mouse_y = mouse_y;
    mouse_report_.prev_x_pos = prev_mouse_x;
    mouse_report_.prev_y_pos = prev_mouse_y;
    buttons_ = SDL_GetMouseState(&mouse_x, &mouse_y);
    mouse_report_.x_position = mouse_x;
    mouse_report_.y_position = mouse_y;

    if ((buttons_ & SDL_BUTTON_LMASK) != 0) {
      mouse_report_.left_clicked = true;
    }
    if ((buttons_ & SDL_BUTTON_LMASK) == 0) {
      mouse_report_.left_clicked = false;
    }
    if ((buttons_ & SDL_BUTTON_RMASK) != 0) {
      mouse_report_.right_clicked = true;
    }
    if ((buttons_ & SDL_BUTTON_RMASK) == 0) {
      mouse_report_.right_clicked = false;
    }

    return mouse_report_;
  }


private:
  MouseReport mouse_report_;
  int mouse_x;
  int mouse_y;
  int prev_mouse_x;
  int prev_mouse_y;
  uint32_t buttons_;
};

}
}


#endif /* mouse_controller_hpp */
