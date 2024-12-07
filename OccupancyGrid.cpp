
#include "OccupancyGrid.h"
#include "glm/common.hpp"

#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/random.hpp>
#include <iostream>

void OccupancyGrid::buildGrid(const GridSettings& settings){
    gridSettings = settings;
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
    parents.clear();
    boxes.clear();

    /* need to use parent indices for this one
    // if provided collision shapes, add them
    for(std::vector<OBB> branch : boxes){
        for(OBB& prism : branch){
            addBox(prism,);
        }
    }
    */
}

void OccupancyGrid::clearGrid(){
    buildGrid(gridSettings);
}

bool OccupancyGrid::addBox(const OBB& box, const uint32_t parentBranch){
    while(parents.size() <= box.branchIndex){
        parents.push_back(0);
    }
    parents[box.branchIndex] = parentBranch;

    vec3 gridOrigin = (box.origin + vec3(gridSettings.gridSize)) / gridSettings.cellSize; 
    // get bounding box
    std::array<vec3,8> corners = getCorners(box);
    vec3 minCorner = vec3(gridSettings.gridSize * 10.0f);
    vec3 maxCorner = vec3(-gridSettings.gridSize * 10.0f);
    for(vec3& corner : corners){
        if(corner.x < minCorner.x) minCorner.x = corner.x;
        if(corner.y < minCorner.y) minCorner.y = corner.y;
        if(corner.z < minCorner.z) minCorner.z = corner.z;

        if(corner.x > maxCorner.x) maxCorner.x = corner.x;
        if(corner.y > maxCorner.y) maxCorner.y = corner.y;
        if(corner.z > maxCorner.z) maxCorner.z = corner.z;
    }
    
    vec3 minCornerGrid = glm::floor((minCorner + vec3(gridSettings.gridSize)) / gridSettings.cellSize);
    vec3 maxCornerGrid = glm::ceil((maxCorner + vec3(gridSettings.gridSize)) / gridSettings.cellSize);

    std::vector<uint32_t> voxelsWithBox;
    uint32_t gridIndex = 0;
    for(uint16_t x = minCornerGrid.x; x <= maxCornerGrid.x; x++){
        for(uint16_t y = minCornerGrid.y; y <= maxCornerGrid.y; y++){
            for(uint16_t z = minCornerGrid.z; z <= maxCornerGrid.z; z++){
                // get a list of voxels that the box intercepts
                if(collideWithVoxel(box, x, y, z)){
                    voxelsWithBox.push_back(gridIndex);
                }
                gridIndex++;
            }
        }
    }

    std::unordered_set<uint32_t> checkedParents;
    for(uint32_t voxel : voxelsWithBox){
        GridCell cell = grid[voxel];
        for(uint8_t occupantIndex = 0; occupantIndex < MAX_OCCUPANTS; occupantIndex++){
            if(
                cell.checkOccupant[occupantIndex] && 
                cell.occupants[occupantIndex] != box.branchIndex && 
                cell.occupants[occupantIndex] != parentBranch
            ){
                // check all boxes of branch
                for(OBB& otherBox : boxes[occupantIndex]){
                    if(collideWithPrism(box, otherBox, false)){
                        return false;
                    }
                }
            }
        }
    }

    while(boxes.size() <= box.branchIndex){
        boxes.push_back(std::vector<OBB>()); 
    }
    boxes[box.branchIndex].push_back(box);

    // place OBB in grid
    for(uint32_t voxel : voxelsWithBox){
        // check if branch is already in voxel
        bool branchInVoxel = false;
        for(uint8_t occupantIndex = 0; occupantIndex < MAX_OCCUPANTS; occupantIndex++){
            if(grid[voxel].checkOccupant[occupantIndex] && grid[voxel].occupants[occupantIndex] == box.branchIndex){
                branchInVoxel = true;
                break;
            }
        }
            
        // add branch to voxel array
        if(!branchInVoxel){ 
            for(uint8_t occupantIndex = 0; occupantIndex < MAX_OCCUPANTS; occupantIndex++){
                if(!grid[voxel].checkOccupant[occupantIndex]){
                    grid[voxel].occupants[occupantIndex] = box.branchIndex;
                    grid[voxel].checkOccupant[occupantIndex] = true;
                    break;
                } 
            }
        }
    }

    return true;
}

bool OccupancyGrid::collideWithPrism(const OBB& box1, const OBB& box2, const bool voxelCheck){
    if(!voxelCheck){
        // ignore cases
        if(box1.branchIndex == box2.branchIndex){
            return false;
        }
        else if(parents[box1.branchIndex] == parents[box2.branchIndex]){
            return false;
        }
        else if(parents[box1.branchIndex] == box2.branchIndex){
            return false;
        }
        else if(box1.branchIndex == parents[box2.branchIndex]){
            return false;
        }
    }

    const vec3 diff = box2.origin - box1.origin;
    return !(
        separatingPlaneExists(diff, box1.xDimension, box1, box2) ||
        separatingPlaneExists(diff, box1.yDimension, box1, box2) ||
        separatingPlaneExists(diff, box1.zDimension, box1, box2) ||

        separatingPlaneExists(diff, box2.xDimension, box1, box2) ||
        separatingPlaneExists(diff, box2.yDimension, box1, box2) ||
        separatingPlaneExists(diff, box2.zDimension, box1, box2) ||

        separatingPlaneExists(diff, glm::cross(box1.xDimension, box2.xDimension), box1, box2) ||
        separatingPlaneExists(diff, glm::cross(box1.xDimension, box2.yDimension), box1, box2) ||
        separatingPlaneExists(diff, glm::cross(box1.xDimension, box2.zDimension), box1, box2) ||

        separatingPlaneExists(diff, glm::cross(box1.yDimension, box2.xDimension), box1, box2) ||
        separatingPlaneExists(diff, glm::cross(box1.yDimension, box2.yDimension), box1, box2) ||
        separatingPlaneExists(diff, glm::cross(box1.yDimension, box2.zDimension), box1, box2) ||

        separatingPlaneExists(diff, glm::cross(box1.zDimension, box2.xDimension), box1, box2) ||
        separatingPlaneExists(diff, glm::cross(box1.zDimension, box2.yDimension), box1, box2) ||
        separatingPlaneExists(diff, glm::cross(box1.zDimension, box2.zDimension), box1, box2)
    );
}

bool OccupancyGrid::separatingPlaneExists(const vec3& diff, const vec3& planeNormal, const OBB& box1, const OBB& box2){
    return fabs(glm::dot(diff, planeNormal)) >
    (
        fabs(glm::dot(box1.xDimension * box1.dimensionSizes.x, planeNormal)) +
        fabs(glm::dot(box1.yDimension * box1.dimensionSizes.y, planeNormal)) +
        fabs(glm::dot(box1.zDimension * box1.dimensionSizes.z, planeNormal)) +
        fabs(glm::dot(box2.xDimension * box2.dimensionSizes.x, planeNormal)) +
        fabs(glm::dot(box2.yDimension * box2.dimensionSizes.y, planeNormal)) +
        fabs(glm::dot(box2.zDimension * box2.dimensionSizes.z, planeNormal))
    );
}

bool OccupancyGrid::collideWithVoxel(const OBB& box, const uint16_t x, const uint16_t y, const uint16_t z){
    OBB cellbox;
    cellbox.dimensionSizes = vec3(gridSettings.cellSize/2.0f);
    cellbox.origin = vec3(
        -gridSettings.gridSize + gridSettings.cellSize * x,
        -gridSettings.gridSize + gridSettings.cellSize * y,
        -gridSettings.gridSize + gridSettings.cellSize * z
    ) + cellbox.dimensionSizes;
    cellbox.xDimension = vec3(1,0,0);
    cellbox.yDimension = vec3(0,1,0);
    cellbox.zDimension = vec3(0,0,1);

    return collideWithPrism(box, cellbox, true);
}

std::array<glm::vec3, 8> OccupancyGrid::getCorners(const OBB& box){
    std::array<vec3, 8> corners = std::array<vec3,8>();
    uint8_t cornerIndex = 0;
    for(uint8_t i = 0; i < 2; i++){
        for(uint8_t j = 0; j < 2; j++){
            for(uint8_t k = 0; k < 2; k++){
                corners[cornerIndex] = box.origin - box.dimensionSizes
                        + (box.xDimension * static_cast<float>(i) * box.dimensionSizes.x * 2.0f) +  
                        + (box.yDimension * static_cast<float>(j) * box.dimensionSizes.y * 2.0f) +  
                        + (box.zDimension * static_cast<float>(k) * box.dimensionSizes.z * 2.0f); 
                cornerIndex++;
            } 
        } 
    }  
    return corners;
}
