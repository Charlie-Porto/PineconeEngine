#ifndef virtual_keyboard_hpp
#define virtual_keyboard_hpp

#include <iostream>
/* include Simulation.h so that the renderer can be accessed */
#include "PineconeSDL2/core/Simulation.hpp"
#include "joystick_report.hpp"

namespace pce {

class VirtualKeyboard 
{
public:
    VirtualKeyboard();
    ~VirtualKeyboard(){};
    JoystickReport check_buttons();
private:
    JoystickReport joystick_; 
}; 

}


#endif /* virtual_keyboard_hpp */
