#ifndef Simulation_h
#define Simulation_h

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
class for initializing SDL and managing the SDL event loop
-----------------------------------------------------------------*/
#include <vector>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

namespace pce {

class Simulation {
public:

  void Init(const char* title, int xpos, int ypos, int width, int height, bool fullscreen);
  void Init(const char* title, int width, int height);
  void Init(const char* title);

  void HandleEvents();
  void ClearFrameEvents();

  void Render();
  void ClearRenderer();
  void Clean();
  bool Running() { return isRunning; }

  static SDL_Renderer* renderer;
  static SDL_Event event;
  static std::vector<SDL_Event> frame_events;
  static bool isRunning;

private:
  int cnt_ = 0;
  SDL_Window *window_;
};

}





#endif /* Simulation_h */
