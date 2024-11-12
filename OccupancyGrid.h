#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <json.hpp>
#include <map>
#include <string>
#include <vector>
#include <array>
#include <unordered_set>

class OccupancyGrid{
public:
  static const uint8_t MAX_OCCUPANTS = 32;

  // (Just aliases to make notations lighter)
  using vec3 = glm::vec3;
  using vec2 = glm::vec2;
  using mat3x3 = glm::mat3x3;
  using quat = glm::quat;

  struct CollisionPrism{
    vec3 origin;
    vec3 xDimension;
    vec3 yDimension;
    vec3 zDimension;
    uint32_t branchIndex;
  };

  struct GridCell{
    // this structure may be necessary for collisions in the physics engine
    // dynamic arrays will not be able to be passed into compute shader, 
    // and branches may enter and exit grid cells
    std::array<uint32_t, MAX_OCCUPANTS> occupants;
    std::array<bool, MAX_OCCUPANTS> checkOccupant;
  };

  struct GridSettings{
    float gridSize = 5.0f;
    float cellSize = 0.2f;
  };

  void buildGrid(const GridSettings& settings);
  std::vector<CollisionPrism> getCollisions(const CollisionPrism& newBranch);
  void addCollisionShape(const CollisionPrism& collisionShape);


private:  
  size_t gridDimension = 0;
  std::vector<GridCell> grid;
  std::vector<std::unordered_set<size_t>> branchToGridCells;
  std::vector<std::vector<CollisionPrism>> collisionShapes;

  GridSettings gridSettings;

  bool collideWithPrism(const CollisionPrism& collisionShape1, const CollisionPrism& collisionShape2);
  bool collideWithVoxels(const CollisionPrism& collisionShape, const GridCell& cell);
  std::array<vec3, 8> getCorners(const CollisionPrism& collisionShape);
  bool pointInPrism(const CollisionPrism& collisionShape, const vec3& point);
};
