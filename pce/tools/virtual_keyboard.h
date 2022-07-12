#ifndef virtual_keyboard_h
#define virtual_keyboard_h

#include <iostream>

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include "../core/Simulation.h"      // for events 
#include "joystick_report.cpp"

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


#endif /* virtual_keyboard_h */
