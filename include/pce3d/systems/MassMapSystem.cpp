#ifndef MassMapSystem_cpp
#define MassMapSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system for maintaining rigid body mass location maps. this helps determine
the effects of collisions.
-----------------------------------------------------------------*/

#include <pcecs/ecs/System.cpp>

#include "functions/massMapFunctions.hpp"

extern ControlPanel control;

namespace pce3d {
class MassMapSystem : public ISystem {
public:

  void DoPreLoopSetup()
  {
    /* create mass map for each entity */
    for (auto const& entity : entities)
    {
      auto& mass_dist = control.GetComponent<pce::MassDistribution>(entity);
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity); 
      auto const& position = control.GetComponent<pce::Position>(entity); 

      /* if particle or sphere, only 1 mass zone */
      if (rigid_object.radius != 0)
      {
        mass_dist.mass_zones[1] = rigid_object.vertices.at(1);
        mass_dist.zone_mass = rigid_object.mass;
        continue;
      }

      /* if NOT particle or sphere, do ordinary mass zone calc */
      const std::vector<glm::dvec3> surface_zones = pce3d::massmap::findAllSurfaceMassZones(
        rigid_object.face_vertex_map, rigid_object.vertices);
      
      pce3d::massmap::findAllMassZonesGivenSurfaceZones(
        position.actual_center_of_mass,
        surface_zones,
        mass_dist.mass_zones,
        mass_dist.coordinates_to_zone_map,
        mass_dist.mass_zones_distance_from_center_of_mass);
    }
  }


  void UpdateEntities() 
  {
    for (auto const& entity : entities) 
    {
      auto& mass_dist = control.GetComponent<pce::MassDistribution>(entity);

      if (mass_dist.need_to_recalculate_velocities)
      {
        /* recalc velocities */
      }

    }
  }

private:

};
}
#endif /* MassMapSystem_cpp */
