#ifndef Simulation_cpp
#define Simulation_cpp

#include <iostream>
#include "Simulation.h"

namespace pce {

/* initialize static variables */
SDL_Renderer* Simulation::renderer = nullptr;
std::vector<SDL_Event> Simulation::frame_events;
SDL_Event Simulation::event;
bool Simulation::isRunning = true;


void Simulation::Init(const char* title, int xpos, int ypos, int width, int height, bool fullscreen) {
  int flags = 0;  
  if (fullscreen) { flags = SDL_WINDOW_FULLSCREEN; }

  if (SDL_Init(SDL_INIT_EVERYTHING) == 0) {
    std::cout << "Subsystems initialized" << '\n';

    window_ = SDL_CreateWindow(title, xpos, ypos, width, height, flags);
    if(window_) { std::cout << "Window created" << std::endl; }

    renderer = SDL_CreateRenderer(window_, -1, 0) ;
    if(renderer) {
      SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
      std::cout << "Renderer created" << std::endl;
    }
  }
}

void Simulation::Init(const char* title) {
  Simulation::Init(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1000, 672, false);
}


void Simulation::HandleEvents() {
  while (SDL_PollEvent(&event)) {
    frame_events.push_back(event);
  }

  for (auto const& m_event : frame_events) {
    switch (m_event.type) {
      case SDL_QUIT:
        isRunning = false;
        break;
      default:
        break;
    }
  }
}


void Simulation::ClearFrameEvents() {
  frame_events.clear();
}


void Simulation::Render() {
  SDL_RenderPresent(renderer);
}

void Simulation::ClearRenderer() {
  SDL_RenderClear(renderer);
}


void Simulation::Clean() {
  SDL_DestroyWindow( window_ );
  SDL_DestroyRenderer( renderer );
  SDL_Quit();
  std::cout << "Simulation cleaned" << '\n';
}



}



#endif /* Simulation_cpp */
