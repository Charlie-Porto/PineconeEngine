#ifndef CoreManager_cpp
#define CoreManager_cpp

#include "CoreManager.h"

namespace pce {

int pce::CoreManager::SCREEN_X = 1000;
int pce::CoreManager::SCREEN_Y = 672;

CoreManager::CoreManager() {
  simulation_ = new Simulation();
  simulation_->Init("Pinecone Engine", SCREEN_X, SCREEN_Y);
  FPS_ = 60;
  frameDelay_ = 1000 / FPS_;
}

CoreManager::CoreManager(const char* title) {
  simulation_ = new Simulation();
  simulation_->Init("title", SCREEN_X, SCREEN_Y);
  FPS_ = 60;
  frameDelay_ = 1000 / FPS_;
  SetScreenParameters(1000, 672);
}

CoreManager::CoreManager(const char* title, int screen_x, int screen_y) {
  simulation_ = new Simulation();
  simulation_->Init("title", screen_x, screen_y);
  FPS_ = 60;
  frameDelay_ = 1000 / FPS_;
  SetScreenParameters(screen_x, screen_y);
}


CoreManager::~CoreManager() {
  simulation_->Clean();
  delete simulation_;
}


void CoreManager::DoCorePreUpdate() {
  frameStart_ = SDL_GetTicks();       
  simulation_->HandleEvents();
  simulation_->ClearRenderer();
}


void CoreManager::DoCorePostUpdate() {
  simulation_->Render();
  simulation_->ClearFrameEvents();
  
  frameTime_ = SDL_GetTicks() - frameStart_;
  if (frameDelay_ > frameTime_) {
    SDL_Delay(frameDelay_ - frameTime_);
  }
}
  
void CoreManager::SetScreenParameters(int x, int y) {
  SCREEN_X = x;
  SCREEN_Y = y;
}

}




#endif /* CoreManager_cpp */
