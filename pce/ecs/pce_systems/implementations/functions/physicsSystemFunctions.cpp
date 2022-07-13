#ifndef physicsSystemFunctions_cpp
#define physicsSystemFunctions_cpp

/*----------------------------------------------------------------|
--------------------- Module Description -------------------------|
free functions to assist the Physics System
-----------------------------------------------------------------*/

#include <cmath>

#include <glm/vec3.hpp>
#include <glm/geometric.hpp>

#include <ezprint.cpp>

#include "MapBuilderSystem.cpp"
#include "../../constants/static_variables.cpp"

namespace pce {
namespace phys {

uint32_t getEntityByPositionFromMapArray(const glm::dvec3& position) {
  const glm::dvec3 index = position - pce::MapBuilderSystem::origin_index_ - glm::dvec3(0, 0, 10);
  return pce::MapBuilderSystem::map_array_.at(int(index.x), int(index.y), int(index.z));
}

void calculateAirbornePosition(pce::Motion& motion, pce::Orientation& orientation, double time) {
  motion.time_airborne += time;
  ezp::print_labeled_item("time airborne: ", motion.time_airborne);
  const double y_position_a = -9.81*(pow(motion.time_airborne, 2.0)) / 2.0;
  const double y_position_v = motion.initial_velocity.y * motion.time_airborne;
  const double new_y_position = y_position_a + y_position_v + motion.previous_ground_position.y;
  
  const double x_position_v = motion.initial_velocity.x;
  double new_x_position = x_position_v + orientation.previous_position.x;
  if (new_x_position != new_x_position) {
    new_x_position = motion.previous_ground_position.x;
  }

  const double z_position_v = motion.initial_velocity.z;
  double new_z_position = z_position_v + orientation.previous_position.z;
  if (new_z_position != new_z_position) {
    new_z_position = motion.previous_ground_position.z;
  }
  orientation.position = glm::dvec3(new_x_position, new_y_position, new_z_position);
}

void checkForMovementObstructions(pce::Orientation& orientation, pce::Motion& motion) {
  const glm::dvec3 dir_travel = orientation.position - orientation.previous_position;
  if (dir_travel.x != 0 || dir_travel.y != 0 || dir_travel.z != 0) {
    ezp::print_item("--------");
    const double prev_x = orientation.previous_position.x;
    const double prev_y = orientation.previous_position.y;
    const double prev_z = orientation.previous_position.z;

    const double new_x = orientation.position.x;
    const double new_y = orientation.position.y;
    const double new_z = orientation.position.z;

    /* check for obstruction in z component of motion */
    const glm::dvec3 z_dir_index = MapBuilderSystem::origin_index_ - glm::dvec3(prev_x, -(prev_y-2), new_z-10.0);
    if (z_dir_index.z < 0 || z_dir_index.z > pce::map_depth_z) {
      orientation.position.z = orientation.previous_position.z; 
    } else if (MapBuilderSystem::map_array_.at(z_dir_index.x, z_dir_index.y, z_dir_index.z) > 0) {
      orientation.position.z = orientation.previous_position.z; 
    }

    /* check for obstruction in x component of motion */
    const glm::dvec3 x_dir_index = MapBuilderSystem::origin_index_ - glm::dvec3(new_x, -(prev_y-2), prev_z-10.0);
    // vezp::print_labeled_dvec3("x index: ", x_dir_index);
    if (x_dir_index.x < 0 || x_dir_index.x > pce::map_width_x) {
      orientation.position.x = orientation.previous_position.x; 
    } else if (MapBuilderSystem::map_array_.at(x_dir_index.x, x_dir_index.y, x_dir_index.z) > 0) {
      orientation.position.x = orientation.previous_position.x; 
    }

    /* check for obstruction in y component of motion */
    const glm::dvec3 y_dir_index = MapBuilderSystem::origin_index_ - glm::dvec3(prev_x, -(new_y-global_const::player_block_height), prev_z-10.0);
    if (y_dir_index.y < 0 || y_dir_index.y > pce::map_height_y) {
      orientation.position.y = orientation.previous_position.y * 0.9; 
    } else if (MapBuilderSystem::map_array_.at(y_dir_index.x, y_dir_index.y, y_dir_index.z) > 0) {
      orientation.position.y = double(ceil(orientation.previous_position.y)); 
      motion.is_airborne = false;
      motion.time_airborne = 0.0;
    }
  } 
  motion.travel_direction = dir_travel;
  orientation.previous_position = orientation.position;
  

  // vezp::print_labeled_dvec3("travel direction", motion.travel_direction);
  const glm::dvec3 motion_uvec = glm::normalize(motion.travel_direction);
  if (motion_uvec.x || motion_uvec.y || motion_uvec.z)  {
    motion.speed = sqrt(glm::dot(motion.travel_direction, motion.travel_direction));
    // ezp::print_labeled_item("speed: ", motion.speed);
  } else {
    motion.speed = 0;
  }
}

}
}



#endif /* physicsSystemFunctions_cpp */
