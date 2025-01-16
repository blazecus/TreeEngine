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
  static const uint8_t MAX_OCCUPANTS = 64;

  // (Just aliases to make notations lighter)
  using vec3 = glm::vec3;
  using vec2 = glm::vec2;
  using mat3x3 = glm::mat3x3;
  using quat = glm::quat;

  struct OBB{
    vec3 origin;
    vec3 xDimension;
    vec3 yDimension;
    vec3 zDimension;
    vec3 dimensionSizes;
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
    float cellSize = 0.3f;
  };

  void buildGrid(const GridSettings& settings);
  void clearGrid();
  std::vector<OBB> getCollisions(const OBB& newBranch);
  bool addBox(const OBB& box, const uint32_t parentBranch);


private:  
  size_t gridDimension = 0;
  std::vector<GridCell> grid;
  std::vector<std::unordered_set<size_t>> branchToGridCells;
  std::vector<std::vector<OBB>> boxes;
  std::vector<uint32_t> parents;

  GridSettings gridSettings;

  bool separatingPlaneExists(const vec3& diff, const vec3& planeNormal, const OBB& box1, const OBB& box2);
  bool collideWithPrism(const OBB& box1, const OBB& box2, const bool voxelCheck);
  bool collideWithVoxel(const OBB& box, const uint16_t x, const uint16_t y, const uint16_t z);
  std::array<vec3, 8> getCorners(const OBB& box);
};
