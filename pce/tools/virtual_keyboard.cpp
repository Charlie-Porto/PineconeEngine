#ifndef virtual_keyboard_cpp
#define virtual_keyboard_cpp

#include <iostream>

#include "virtual_keyboard.h"

namespace pce {

VirtualKeyboard::VirtualKeyboard() {
    joystick_.R_pressed = false;
    joystick_.L_pressed = false;
    joystick_.Down_pressed = false;
    joystick_.Up_pressed = false;
    joystick_.W_pressed = false;
    joystick_.S_pressed = false;
    joystick_.A_pressed = false;
    joystick_.D_pressed = false;
    joystick_.r_pressed = false;
    joystick_.F_pressed = false;
    joystick_.X_pressed = false;
    joystick_.SPACE_pressed = false;
}

JoystickReport VirtualKeyboard::check_buttons()
{
    /* poll all events */
    for (int i = 0; i < Simulation::frame_events.size(); i++)
    {
        /* check specifically for key press event */
        if ( Simulation::frame_events[i].type == SDL_KEYDOWN )
        {
            switch ( Simulation::frame_events[i].key.keysym.sym )
            {
                case SDLK_RIGHT:
                    std::cout<<"PRESS: R" <<'\n';
                    joystick_.R_pressed = true;
                    break;
                case SDLK_LEFT:
                    std::cout<<"PRESS: L" <<'\n';
                    joystick_.L_pressed = true;
                    break;
                case SDLK_UP:
                    std::cout<<"PRESS: UP" <<'\n';
                    joystick_.Up_pressed = true;
                    break;
                case SDLK_DOWN:
                    std::cout<<"PRESS: DOWN" <<'\n';
                    joystick_.Down_pressed = true;
                    break;
                case SDLK_w:
                    std::cout<<"PRESS: W" <<'\n';
                    joystick_.W_pressed = true;
                    break;
                case SDLK_s:
                    std::cout<<"PRESS: S" <<'\n';
                    joystick_.S_pressed = true;
                    break;
                case SDLK_a:
                    std::cout<<"PRESS: A" <<'\n';
                    joystick_.A_pressed = true;
                    break;
                case SDLK_d:
                    std::cout<<"PRESS: D" <<'\n';
                    joystick_.D_pressed = true;
                    break;
                case SDLK_x:
                    std::cout<<"PRESS: X" <<'\n';
                    joystick_.X_pressed = true;
                    break;
                case SDLK_r:
                    std::cout<<"PRESS: r" <<'\n';
                    joystick_.r_pressed = true;
                    break;
                case SDLK_f:
                    std::cout<<"PRESS: F" <<'\n';
                    joystick_.F_pressed = true;
                    break;
                case SDLK_SPACE:
                    std::cout<<"PRESS: SPACE" <<'\n';
                    joystick_.SPACE_pressed = true;
                    break;
                default:
                    break;
            } 
        }

        else if ( Simulation::frame_events[i].type == SDL_KEYUP )
        {
            switch ( Simulation::frame_events[i].key.keysym.sym )
            {
                case SDLK_RIGHT:
                    std::cout<<"LIFT: R" <<'\n';
                    joystick_.R_pressed = false;
                    break;
                case SDLK_LEFT:
                    std::cout<<"LIFT: L" <<'\n';
                    joystick_.L_pressed = false;
                    break;
                case SDLK_UP:
                    std::cout<<"LIFT: UP" <<'\n';
                    joystick_.Up_pressed = false;
                    break;
                case SDLK_DOWN:
                    std::cout<<"LIFT: DOWN" <<'\n';
                    joystick_.Down_pressed = false;
                    break;
                case SDLK_w:
                    std::cout<<"LIFT: W" <<'\n';
                    joystick_.W_pressed = false;
                    break;
                case SDLK_s:
                    std::cout<<"LIFT: S" <<'\n';
                    joystick_.S_pressed = false;
                    break;
                case SDLK_a:
                    std::cout<<"LIFT: A" <<'\n';
                    joystick_.A_pressed = false;
                    break;
                case SDLK_d:
                    std::cout<<"LIFT: D" <<'\n';
                    joystick_.D_pressed = false;
                    break;
                case SDLK_x:
                    std::cout<<"LIFT: X" <<'\n';
                    joystick_.X_pressed = false;
                    break;
                case SDLK_r:
                    std::cout<<"LIFT: r" <<'\n';
                    joystick_.r_pressed = false;
                    break;
                case SDLK_f:
                    std::cout<<"LIFT: F" <<'\n';
                    joystick_.F_pressed = false;
                    break;
                case SDLK_SPACE:
                    std::cout<<"LIFT: SPACE" <<'\n';
                    joystick_.SPACE_pressed = false;
                    break;
                default:
                    break;
            }
        }
    }
    return joystick_; 
}


}

#endif /* virtual_keyboard_cpp */
