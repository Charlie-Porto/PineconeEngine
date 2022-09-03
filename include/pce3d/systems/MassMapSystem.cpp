#ifndef MassMapSystem_cpp
#define MassMapSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system for maintaining rigid body mass location maps. this helps determine
the effects of collisions.
-----------------------------------------------------------------*/

#include <pcecs/ecs/System.cpp>

#include "functions/massMapFunctions.hpp"
#include "functions/radarFunctions.hpp"

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


  void drawMapPointsInSpace(const glm::dquat& cam_versor, const glm::dvec3& cam_transform) {
    for (auto const& entity : entities)
    {
      auto const& mass_dist = control.GetComponent<pce::MassDistribution>(entity);
      for (auto const& [id, point] : mass_dist.mass_zones) {
        glm::dvec3 rotated_point = point - cam_transform;
        double distance = sqrt(glm::dot(rotated_point, rotated_point));
        rotated_point = pce::rotateVector3byQuaternion(rotated_point, cam_versor);     
        const glm::dvec3 vs_intersection = glm::normalize(rotated_point);
        const glm::dvec2 pixel = radar::convertPointOnViewSphereToPixel(vs_intersection, true, false);
        std::vector<int> color = {12, 20, 200, 255};
        pce::quickdraw::drawCircle(pixel, 10.0 / distance, color);
      }
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
