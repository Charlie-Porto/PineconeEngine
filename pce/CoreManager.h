#ifndef CoreManager_h
#define CoreManager_h

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
class to handle all function calls to the Simulation class
-----------------------------------------------------------------*/

#include "core/Simulation.h"
#include <ezprint.cpp>

namespace pce {

class CoreManager {
public:
  CoreManager();
  ~CoreManager();

  void DoCorePreUpdate();
  void DoCorePostUpdate();
  double getFrameStartTime () const { return double(frameStart_/1000.0); }
  bool Running() const { return simulation_->isRunning; }

private:
  Simulation* simulation_;
  int FPS_;
  int frameDelay_;
  int frameStart_;
  int frameTime_;

};


}




#endif /* CoreManager_h */
