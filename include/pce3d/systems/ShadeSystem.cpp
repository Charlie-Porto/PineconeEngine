#ifndef ShadeSystem_cpp
#define ShadeSystem_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
system for calculating amount of light reaching object faces (thus the color)
-----------------------------------------------------------------*/

#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include <pcecs/ecs/System.cpp>
#include <ezprint.cpp>
#include "functions/shadeFunctions.hpp"

extern ControlPanel control;
namespace pce3d {


class ShadeSystem : public ISystem {
public:

  void UpdateEntities() {
    for (auto const& entity : entities) {
      auto const& rigid_object = control.GetComponent<pce::RigidObject>(entity);
      auto& face_shade = control.GetComponent<pce::FaceShade>(entity); 

      for (auto& [face, vertices] : rigid_object.face_vertex_map) {
        glm::dvec3 vertex_a = rigid_object.vertices.at(vertices[1]) - rigid_object.vertices.at(vertices[0]);
        glm::dvec3 vertex_b = rigid_object.vertices.at(vertices[2]) - rigid_object.vertices.at(vertices[0]);
        const glm::dvec3 normal_vect = glm::cross(vertex_a, vertex_b);
        face_shade.face_shade_map[face] = shade::calculateFaceBrightness(LIGHT_FLOW_DIRECTION_, normal_vect);
      }
    }
  }

private:
  const glm::dvec3 LIGHT_FLOW_DIRECTION_ = glm::dvec3(-0.1, -1.0, -0.2);

};
}
#endif /* ShadeSystem_cpp */
