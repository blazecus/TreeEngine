
#include "OccupancyGrid.h"
#include "glm/common.hpp"

#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/random.hpp>

void OccupancyGrid::buildGrid(const GridSettings& settings){
    // initiate grid vector of given size
    gridDimension = (2 * gridSettings.gridSize / gridSettings.cellSize) + 1;
    // save memory by enforcing correct vector size
    grid.clear();
    grid.reserve(gridDimension * gridDimension * gridDimension);
    for(uint32_t x = 0; x < gridDimension; x++){
        for(uint32_t y = 0; y < gridDimension; y++){
            for(uint32_t z = 0; z < gridDimension; z++){
                grid.push_back(GridCell{});
            }
        }
    }

    // if provided collision shapes, add them
    for(std::vector<CollisionPrism> branch : collisionShapes){
        for(CollisionPrism prism : branch){
            addCollisionShape(prism);
        }
    }
}

void OccupancyGrid::addCollisionShape(const CollisionPrism& collisionShape){
    vec3 gridOrigin = (collisionShape.origin + vec3(gridSettings.gridSize)) / gridSettings.cellSize; 


}

bool OccupancyGrid::collideWithPrism(const CollisionPrism& collisionShape1, const CollisionPrism& collisionShape2){

}

bool OccupancyGrid::collideWithVoxels(const CollisionPrism& collisionShape, const GridCell& cell){

}

std::array<glm::vec3, 8> OccupancyGrid::getCorners(const CollisionPrism& collisionShape){
    std::array<vec3, 8> corners;
    uint8_t cornerIndex = 0;
    for(uint8_t i = 0; i < 2; i++){
        for(uint8_t j = 0; j < 2; j++){
            for(uint8_t k = 0; k < 2; k++){
                corners[cornerIndex] = collisionShape.origin 
                                        + (collisionShape.xDimension * static_cast<float>(i)) +  
                                        + (collisionShape.yDimension * static_cast<float>(j)) +  
                                        + (collisionShape.zDimension * static_cast<float>(k)); 
                cornerIndex++;
            } 
        } 
    } 
    
    return corners;
}

bool OccupancyGrid::pointInPrism(const CollisionPrism& collisionShape, const vec3& point){
    vec3 magVec = vec3(collisionShape.xDimension.length, 
                        collisionShape.yDimension.length,
                        collisionShape.zDimension.length);
    magVec *= magVec;
    vec3 relativePoint = point - collisionShape.origin;
    vec3 projection = vec3(
        glm::dot(relativePoint, collisionShape.xDimension),
        glm::dot(relativePoint, collisionShape.yDimension),
        glm::dot(relativePoint, collisionShape.zDimension)
    );

    bool inX = projection.x >= 0 && projection.x < magVec.x; 
    bool inY = projection.y >= 0 && projection.y < magVec.y; 
    bool inZ = projection.z >= 0 && projection.z < magVec.z; 

    return inX && inY && inZ;
}

bool OccupancyGrid::voxelInShape(const size_t cellIndex, const CollisionPrism& collisionShape){
    GridCell cell = grid[cellIndex];
    size_t x = cellIndex / (gridDimension * gridDimension);
    size_t y = (cellIndex % (gridDimension * gridDimension)) / gridDimension;
    size_t z = cellIndex % gridDimension;
    vec3 origin = 
    for(uint8_t i = 0; i < 2; i++){
        for(uint8_t j = 0; j < 2; j++){
            for(uint8_t k = 0; k < 2; k++){

            } 
        } 
    } 
}