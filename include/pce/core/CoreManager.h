#ifndef CoreManager_h
#define CoreManager_h

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
class to handle all function calls to the Simulation class
-----------------------------------------------------------------*/

#include "Simulation.h"

namespace pce {

class CoreManager {
public:
  CoreManager();
  CoreManager(const char* title);
  CoreManager(const char* title, int screen_x, int screen_y);
  ~CoreManager();

  void DoCorePreUpdate();
  void DoCorePostUpdate();
  double getFrameStartTime () const { return double(frameStart_/1000.0); }
  bool Running() const { return simulation_->isRunning; }

  static int SCREEN_X;
  static int SCREEN_Y;

private:
  Simulation* simulation_;
  int FPS_;
  int frameDelay_;
  int frameStart_;
  int frameTime_;

  void SetScreenParameters(int x, int y);
};


}




#endif /* CoreManager_h */
