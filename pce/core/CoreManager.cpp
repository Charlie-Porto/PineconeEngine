#ifndef CoreManager_cpp
#define CoreManager_cpp

#include "CoreManager.h"

namespace pce {

CoreManager::CoreManager() {
  simulation_ = new Simulation();
  simulation_->Init("Pinecone Engine");
  FPS_ = 60;
  frameDelay_ = 1000 / FPS_;
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

}




#endif /* CoreManager_cpp */
