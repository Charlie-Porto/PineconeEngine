#ifndef collisionFunctions_cpp
#define collisionFunctions_cpp

#include "../collisionFunctions.hpp"

namespace pce3d {
namespace collision {

bool determineIfMovementVectorsIndicateCollision(
    glm::dvec3 a_movement_vector
  , glm::dvec3 b_movement_vector
  , const pce::RigidObject& a_rigid_object
  , const pce::RigidObject& b_rigid_object
  , const glm::dvec3& a_center
  , const glm::dvec3& b_center
)
{
  if (a_movement_vector == glm::dvec3(0, 0, 0))
  {
    // a_movement_vector = b_rigid_object.vertices.at(1) - a_rigid_object.vertices.at(1);
    a_movement_vector = b_center - a_center;
  }
  else if (b_movement_vector == glm::dvec3(0, 0, 0))
  {
    // b_movement_vector = a_rigid_object.vertices.at(1) - b_rigid_object.vertices.at(1);
    b_movement_vector = a_center - b_center;
  }

  const double directions_angle = abs(pce3d::maths::calculateAngleDegreesBetweenVectors(
    a_movement_vector, b_movement_vector));
  std::cout << "original directions angle: " << directions_angle << '\n';

  if (directions_angle > 90.0)
  {
    std::cout << "Directions angle: " << directions_angle << '\n';
    std::cout << "DIRECTIONS ANGLE IS > 90" << '\n';

    // const double distance_a = pce3d::maths::calcMagV3(a_center - b_center);

    // const glm::dvec3 point_b = (a_center + glm::normalize(a_movement_vector) * 0.01);
    // const double distance_b = pce3d::maths::calcMagV3(point_b - b_center);
    // std::cout << "distance a: " << distance_a << '\n';
    // std::cout << "distance b: " << distance_b << '\n';

    // if (distance_a - distance_b > -.006)
    // {
      // return true;
    // }
    // else 
    // {
      // return false;
    // }
    return true;
  }
  else
  {
    /* if NOT POINTING TOWARD EACH OTHER, check if longer vector is behind shorter; if so, collision. */
    glm::dvec3 smaller = a_movement_vector;
    glm::dvec3 larger = b_movement_vector;
    glm::dvec3 scenter = a_center;
    glm::dvec3 lcenter = b_center;
    if (pce3d::maths::calcMagV3(a_movement_vector) > pce3d::maths::calcMagV3(b_movement_vector))
    {
      smaller = b_movement_vector;
      larger = a_movement_vector;
      lcenter = a_center;
      scenter = b_center;
      std::cout << "smaller is b_movement_vector" << '\n';
    }

    const double distance_a = pce3d::maths::calcMagV3(lcenter - scenter);
    const glm::dvec3 point_b = lcenter + glm::normalize(larger) * 0.001;
    const double distance_b = pce3d::maths::calcMagV3(point_b - scenter);
    std::cout << "distance a: " << distance_a << '\n';
    std::cout << "distance b: " << distance_b << '\n';

    // if (distance_a - distance_b > -.006)
    if (distance_a - distance_b > -.0001)
    {
      return true;
    }
    else 
    {
      return false;
    }


    // std::cout << "before: a_movement_vector: "
    //           << a_movement_vector.x << ", "
    //           << a_movement_vector.y << ", "
    //           << a_movement_vector.z << '\n';
    // std::cout << "before: b_movement_vector: "
    //           << b_movement_vector.x << ", "
    //           << b_movement_vector.y << ", "
    //           << b_movement_vector.z << '\n';

    // a_movement_vector = a_movement_vector - smaller;
    // b_movement_vector = b_movement_vector - smaller;
    
    // std::cout << "after: a_movement_vector: "
    //           << a_movement_vector.x << ", "
    //           << a_movement_vector.y << ", "
    //           << a_movement_vector.z << '\n';
    // std::cout << "after: b_movement_vector: "
    //           << b_movement_vector.x << ", "
    //           << b_movement_vector.y << ", "
    //           << b_movement_vector.z << '\n';

    // if (a_movement_vector == glm::dvec3(0, 0, 0))
    // {
    //   // a_movement_vector = b_rigid_object.vertices.at(1) - a_rigid_object.vertices.at(1);
    //   a_movement_vector = b_center - a_center;
    // }
    // if (b_movement_vector == glm::dvec3(0, 0, 0))
    // {
    //   // b_movement_vector = a_rigid_object.vertices.at(1) - b_rigid_object.vertices.at(1);
    //   b_movement_vector = a_center - b_center;
    // }
    // std::cout << "double after: a_movement_vector: "
    //           << a_movement_vector.x << ", "
    //           << a_movement_vector.y << ", "
    //           << a_movement_vector.z << '\n';
    // std::cout << "double after: b_movement_vector: "
    //           << b_movement_vector.x << ", "
    //           << b_movement_vector.y << ", "
    //           << b_movement_vector.z << '\n';

    // const double directions_angle = pce3d::maths::calculateAngleDegreesBetweenVectors(
    //   a_movement_vector, b_movement_vector);
    
    // std::cout << "Directions angle: " << directions_angle << '\n';

    // if (directions_angle > 90.0)
    // {
    //   const double distance_a = pce3d::maths::calcMagV3(lcenter - scenter);
    //   std::cout << "distance a: " << distance_a << '\n';

    //   const glm::dvec3 point_b = (lcenter + glm::normalize(larger) * 0.01);
    //   const double distance_b = pce3d::maths::calcMagV3(point_b - scenter);
    //   std::cout << "distance b: " << distance_b << '\n';

    //   if (distance_a > distance_b)
    //   {
    //     return true;
    //   }
    //   else 
    //   {
    //     return false;
    //   }
    // }
  }
  return false;
}



std::pair<bool, glm::dvec3> determineIfParticlesAreCollidingAndWhere(
    const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
)
{
  const double centers_distance = pce3d::maths::calculateDistanceBetweenVectors(
    a_rigid_object.vertices.at(1), b_rigid_object.vertices.at(1));
  
  const double radius_sum = a_rigid_object.radius + b_rigid_object.radius;

  if (radius_sum < centers_distance)
  {
    return std::make_pair(false, glm::dvec3(0, 0, 0));
  }
  if (a_motion.speed == 0 && b_motion.speed == 0)
  {
    return std::make_pair(false, glm::dvec3(0, 0, 0));
  }

  /* proceeding only if touching; now to see if moving toward each other netly*/
  glm::dvec3 a_direction = a_motion.direction * a_motion.speed;
  glm::dvec3 b_direction = b_motion.direction * b_motion.speed;
  
  if (determineIfMovementVectorsIndicateCollision(
    a_direction, b_direction, a_rigid_object, b_rigid_object, 
    a_rigid_object.vertices.at(1), b_rigid_object.vertices.at(1)))
  {
    return std::make_pair(true, a_rigid_object.vertices.at(1) 
                       + (glm::normalize(b_rigid_object.vertices.at(1) - a_rigid_object.vertices.at(1))
                       * a_rigid_object.radius));
  }
  return std::make_pair(false, glm::dvec3(0, 0, 0));
}



CollisionReport determineIfParticleIsCollidingWithComplexBodAndWhere(
    const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
  , const pce::Position& b_position
)
{

  auto collision_report = CollisionReport{ .entity_a = entity_a, .entity_b = entity_b };
  /* 0. find complex bod's closest vertex to the particle center */
  uint32_t closest_vertex_id = 1;
  double closest_vertex_distance = 100000.0;
  for (auto const& [id, v] : b_rigid_object.vertices)
  {
    const double distance = pce3d::maths::calculateDistanceBetweenVectors(
      a_rigid_object.vertices.at(1), v);
    
    if (distance < closest_vertex_distance)
    {
      closest_vertex_distance = distance;
      closest_vertex_id = id;
    }
  }

  glm::dvec3 potential_point_of_contact = glm::dvec3(b_rigid_object.vertices.at(closest_vertex_id));
  collision_report.point_of_contact = potential_point_of_contact;
  collision_report.collision_occuring = true;
  collision_report.a_collision_type = collision::vertex;
  collision_report.a_collision_type_area_id = 1;
  
  /* 1. check if particle is colliding with closest vertex */
  if (closest_vertex_distance <= a_rigid_object.radius)
  {
    collision_report.b_collision_type = collision::vertex; 
    collision_report.b_collision_type_area_id = closest_vertex_id;
  }
  else if (closest_vertex_distance > a_rigid_object.radius)
  {
    /* 2. if NOT COLLIDING WITH CLOSEST VERTEX, get closest face */
    uint32_t closest_face_id = 1;
    double closest_face_corner_distance = 10000.0;
    assert(b_rigid_object.vertex_face_corner_map.find(closest_vertex_id) 
        != b_rigid_object.vertex_face_corner_map.end());
    for (auto const& [face_id, face_corner_id] : b_rigid_object.vertex_face_corner_map.at(closest_vertex_id))
    {
      assert(b_rigid_object.face_corner_map.find(face_corner_id) != b_rigid_object.face_corner_map.end());

      const glm::dvec3 face_corner_point = b_rigid_object.face_corner_map.at(face_corner_id);
      const double distance = pce3d::maths::calculateDistanceBetweenVectors(
        a_rigid_object.vertices.at(1), face_corner_point);
      
      if (distance < closest_face_corner_distance)
      {
        closest_face_corner_distance = distance;
        closest_face_id = face_id;
      }
    }

    /* 3. check edges of closest face to determine if intersection is an edge intersection*/
    assert (b_rigid_object.face_edge_map.find(closest_face_id) != b_rigid_object.face_edge_map.end());
    bool is_located_at_edge = false;
    for (auto const& edge_id : b_rigid_object.face_edge_map.at(closest_face_id))
    {
      const uint32_t& a_vertex_id = b_rigid_object.edges.at(edge_id).first;
      const uint32_t& b_vertex_id = b_rigid_object.edges.at(edge_id).second;

      const glm::dvec3& a_vertex = b_rigid_object.vertices.at(a_vertex_id);
      const glm::dvec3& b_vertex = b_rigid_object.vertices.at(b_vertex_id);

      const glm::dvec3& closest_edge_point_to_particle_center = pce3d::maths::findClosestPointOnVec3LineToVec3(
        a_vertex, b_vertex, a_rigid_object.vertices.at(1));
      
      const double distance_closest_point_to_center = pce3d::maths::calculateDistanceBetweenVectors(
        closest_edge_point_to_particle_center, a_rigid_object.vertices.at(1));
      
      const bool point_in_line = distance_closest_point_to_center < (a_rigid_object.radius)
        ? true : false;
      
      if (point_in_line)
      {
        potential_point_of_contact = closest_edge_point_to_particle_center;
        collision_report.point_of_contact = potential_point_of_contact;
        collision_report.b_collision_type = collision::edge;
        collision_report.b_collision_type_area_id = edge_id;
        is_located_at_edge = true;
        break;
      }
    }
    if (!is_located_at_edge)
    {
      std::vector<uint32_t> closest_faces{};
      assert(b_rigid_object.vertex_face_corner_map.find(closest_vertex_id) != b_rigid_object.vertex_face_corner_map.end());
      for (auto const& [face, corner] : b_rigid_object.vertex_face_corner_map.at(closest_vertex_id))
      {
        closest_faces.push_back(face);
      }

      /* 3. get closest faces' closest point to particle */
      std::vector<glm::dvec3> closest_face_points{};
      for (size_t i = 0; i < closest_faces.size(); ++i)
      {
        assert(b_rigid_object.face_vertex_map.find(closest_faces[i]) != b_rigid_object.face_vertex_map.end());
        const glm::dvec3 closest_face_point_to_particle = pce3d::maths::calculateClosestPointInPlaneToPoint(
          b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(closest_faces[i])[0]),
          b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(closest_faces[i])[1]),
          b_rigid_object.vertices.at(b_rigid_object.face_vertex_map.at(closest_faces[i])[2]),
          a_rigid_object.vertices.at(1));
        
        closest_face_points.push_back(closest_face_point_to_particle);
      }

      /* 4. determine if distance(closest face's closest point, particle_center) is <= radius */
      std::vector<double> distances{};
      double min_distance = 100000.0;
      glm::dvec3 closest_face_point = closest_face_points[0];
      // uint32_t closest_face_id
      for (size_t i = 0; i < closest_faces.size(); ++i)
      {
        const double distance = pce3d::maths::calculateDistanceBetweenVectors(
          closest_face_points[i], a_rigid_object.vertices.at(1));
        
        distances.push_back(distance);
        if (distance < min_distance)
        {
          min_distance = distance;
          closest_face_id = closest_faces[i];
          closest_face_point = closest_face_points[i];
        }
      }
      
      std::cout << "minimum distance: " << min_distance << '\n';
      if (min_distance > (a_rigid_object.radius + 0.8))
      {
        std::cout << "closest face point does not touch particle" << '\n';
        std::cout << "radius: " << a_rigid_object.radius << '\n';
        collision_report.collision_occuring = false;
        return collision_report;
      }
      else
      {
        potential_point_of_contact = closest_face_point; 
        collision_report.point_of_contact = potential_point_of_contact;
        collision_report.b_collision_type = collision::face;
        collision_report.b_collision_type_area_id = closest_face_id;
      }
    }
  }

  std::cout << "confirmed objects are touching" << '\n';

  /* 5. evaluate direction vectors if objects are, in fact, touching */
  const glm::dvec3 a_direction = a_motion.direction * a_motion.speed;
  glm::dvec3 b_direction = pce3d::physics2::calculateRotationalVelocityOfPointOnObject(
    potential_point_of_contact, 
    b_position.actual_center_of_mass,
    b_motion.rotational_speed,
    b_motion.rotational_axis);

  std::cout << "b_motion.speed: "  << b_motion.speed << '\n';
  std::cout << "b_motion.rotational_speed: "  << b_motion.rotational_speed << '\n';
  // b_direction = glm::normalize(b_direction + b_motion.direction);
  b_direction = b_direction + b_motion.direction;
  std::cout << "a_direction: "
            << a_direction.x << ", "
            << a_direction.y << ", "
            << a_direction.z << '\n';
  std::cout << "b_direction: "
            << b_direction.x << ", "
            << b_direction.y << ", "
            << b_direction.z << '\n';
  // if (determineIfMovementVectorsIndicateCollision(
    // a_direction, b_direction, a_rigid_object, b_rigid_object, 
    // a_rigid_object.vertices.at(1), b_position.actual_center_of_mass))
  // {
    collision_report.point_of_contact = potential_point_of_contact;
    return collision_report;
  // }
  // else 
  // {
    // collision_report.collision_occuring = false;
    // collision_report.point_of_contact = potential_point_of_contact;
    // return collision_report;
  // }
}
 


CollisionReport determineIfComplexBodsAreCollidingAndWhere(
    const glm::ivec3& collision_index
  , const uint32_t entity_a
  , const pce::RigidObject& a_rigid_object
  , const pce::Motion& a_motion
  , const pce::Position& a_position
  , const uint32_t entity_b
  , const pce::RigidObject& b_rigid_object
  , const pce::Motion& b_motion
  , const pce::Position& b_position
)
{
  std::cout << "checking for complexbod-complexbod collision" << '\n';
  auto collision_report = CollisionReport{
    .collision_occuring = true,
    .entity_a = entity_a,
    .entity_b = entity_b
  };
  const double small_dist_threshold = 0.01;

  /* get closest vertices to collision index point */
  const glm::dvec3 collision_index_point = space_map::findPointOfIndex(
    collision_index, pce3d::Core3D::SPACE_MAP_DIMENSIONS, pce3d::Core3D::COLLISION_METER_INDEX_RATIO);
  const uint32_t a_closest_vertex = maths::calculateClosestVertexToPoint(
    collision_index_point, a_rigid_object.vertices);
  const uint32_t b_closest_vertex = maths::calculateClosestVertexToPoint(
    collision_index_point, b_rigid_object.vertices);
  
  /*============ check if vertex intersects vertex ============*/
  const double vertex_distance = maths::calculateDistanceBetweenVectors(
    a_rigid_object.vertices.at(a_closest_vertex), b_rigid_object.vertices.at(b_closest_vertex));
  if (vertex_distance < small_dist_threshold)
  {
    collision_report.point_of_contact = a_rigid_object.vertices.at(a_closest_vertex);
    collision_report.a_collision_type = collision::vertex;
    collision_report.a_collision_type_area_id = a_closest_vertex;
    collision_report.b_collision_type = collision::vertex;
    collision_report.b_collision_type_area_id = b_closest_vertex;
    return collision_report;
  }

  /*============ check if edges intersect edges ============*/
  const std::vector<uint32_t> a_closest_edges = a_rigid_object.vertex_edge_map.at(a_closest_vertex);
  const std::vector<uint32_t> b_closest_edges = b_rigid_object.vertex_edge_map.at(b_closest_vertex);
  for (size_t i = 0; i != a_closest_edges.size(); ++i)
  {
    for (size_t j = 0; j != b_closest_edges.size(); ++j)
    {
      std::pair<double, glm::dvec3> closest_points_distance_and_midpoint 
        = maths::estimateDistanceAndMidPointOfClosestPointsOnLines(
          a_rigid_object.vertices.at(a_rigid_object.edges.at(a_closest_edges[i]).first),
          a_rigid_object.vertices.at(a_rigid_object.edges.at(a_closest_edges[i]).second),
          b_rigid_object.vertices.at(b_rigid_object.edges.at(b_closest_edges[j]).first),
          b_rigid_object.vertices.at(b_rigid_object.edges.at(b_closest_edges[j]).second));
      
      if (closest_points_distance_and_midpoint.first < small_dist_threshold * 10.0)
      {
        collision_report.point_of_contact = closest_points_distance_and_midpoint.second;
        collision_report.a_collision_type = collision::edge;
        collision_report.a_collision_type_area_id = a_closest_edges[i];
        collision_report.b_collision_type = collision::edge;
        collision_report.b_collision_type_area_id = b_closest_edges[j];
        return collision_report;
      }
    }
  }

  /*============ check if closest vertices intersect a face ============*/
  std::cout << "checking for vertex-face collision" << '\n';
  const std::pair<bool, uint32_t> a_vertex_touches_b_face = maths::determineIfVertexTouchesObjectFaces(
    a_rigid_object.vertices.at(a_closest_vertex),
    b_closest_vertex,
    b_rigid_object.vertices,
    b_rigid_object.face_vertex_map,
    b_rigid_object.vertex_face_corner_map,
    small_dist_threshold);
  if (a_vertex_touches_b_face.first)
  {
    std::cout << "a vertex touches b face" << '\n';
    collision_report.point_of_contact = a_rigid_object.vertices.at(a_closest_vertex);
    collision_report.a_collision_type = collision::vertex;
    collision_report.a_collision_type_area_id = a_closest_vertex;
    collision_report.b_collision_type = collision::face;
    collision_report.b_collision_type_area_id = a_vertex_touches_b_face.second;
    return collision_report;
  }
  const std::pair<bool, uint32_t> b_vertex_touches_a_face = maths::determineIfVertexTouchesObjectFaces(
    b_rigid_object.vertices.at(b_closest_vertex),
    a_closest_vertex,
    a_rigid_object.vertices,
    a_rigid_object.face_vertex_map,
    a_rigid_object.vertex_face_corner_map,
    small_dist_threshold);
  if (b_vertex_touches_a_face.first)
  {
    std::cout << "b vertex touches a face" << '\n';
    collision_report.point_of_contact = b_rigid_object.vertices.at(b_closest_vertex);
    collision_report.a_collision_type = collision::face;
    collision_report.a_collision_type_area_id = a_vertex_touches_b_face.second;
    collision_report.b_collision_type = collision::vertex;
    collision_report.b_collision_type_area_id = b_closest_vertex;
    return collision_report;
  }


  /*============ check if closest vertex intersects an edge ============*/
  /* check if b vertex intersects a edge */
  const std::pair<bool, uint32_t> b_vertex_touches = maths::determineIfVertexTouchesObjectEdgesAndWhich(
    b_rigid_object.vertices.at(b_closest_vertex), a_rigid_object.vertices,
    a_rigid_object.edges, small_dist_threshold);
  if (b_vertex_touches.first)
  {
    collision_report.point_of_contact = b_rigid_object.vertices.at(b_closest_vertex);
    collision_report.a_collision_type = collision::edge;
    collision_report.a_collision_type_area_id = b_vertex_touches.second;
    collision_report.b_collision_type = collision::vertex;
    collision_report.b_collision_type_area_id = b_closest_vertex;
    return collision_report;
  }
  /* check if a vertex intersects b edge */
  const std::pair<bool, uint32_t> a_vertex_touches = maths::determineIfVertexTouchesObjectEdgesAndWhich(
    a_rigid_object.vertices.at(a_closest_vertex), b_rigid_object.vertices,
    b_rigid_object.edges, small_dist_threshold);
  if (a_vertex_touches.first)
  {
    collision_report.point_of_contact = a_rigid_object.vertices.at(a_closest_vertex);
    collision_report.a_collision_type = collision::edge;
    collision_report.a_collision_type_area_id = a_closest_vertex;
    collision_report.b_collision_type = collision::vertex;
    collision_report.b_collision_type_area_id = a_vertex_touches.second;
    return collision_report;
  }
  
  collision_report.collision_occuring = false;
  return collision_report;
}


}
}



#endif /* collisionFunctions_cpp */
