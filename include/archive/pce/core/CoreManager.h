#ifndef CoreManager_h
#define CoreManager_h

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
class to handle all function calls to the Simulation class
-----------------------------------------------------------------*/

#include "Simulation.h"
#include "../tools/simple_framerate_timer.hpp"

namespace pce {

class CoreManager {
public:
  CoreManager();
  CoreManager(const char* title);
  CoreManager(const char* title, int screen_x, int screen_y);
  ~CoreManager();

  void DoCorePreUpdate();
  void DoCorePostUpdate();
  void UpdateFramerateTimer();
  void SetScreenParameters(int x, int y);
  bool Running() const { return simulation_->isRunning; }

  /* static items */
  static int SCREEN_X;
  static int SCREEN_Y;
   
  double time_;
  bool USE_TIMER;

private:
  Simulation* simulation_;
  int FPS_;
  int frameDelay_;
  int frameStart_;
  int frameTime_;
  simple_framerate_timer framerate_timer_;
};


}




#endif /* CoreManager_h */
