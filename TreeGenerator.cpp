#include "TreeGenerator.h"
#include "glm/common.hpp"

#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/random.hpp>

#include <fstream>
#include <iostream>

using Branch = TreeGenerator::Branch;
using TreeParameters = TreeGenerator::TreeParameters;
using LSystem = TreeGenerator::LSystem;
using Rule = TreeGenerator::Rule;
using TreeMeshVertex = TreeGenerator::TreeMeshVertex;
using TurtleState = TreeGenerator::TurtleState;
using json = nlohmann::json;
using GridSettings = OccupancyGrid::GridSettings;

// initiateTree: initiates collision grid, rng seed, and sets settings
void TreeGenerator::initiateTree(const TreeParameters& params, const LSystem& ls) {

  treeParameters = params;
  lSystem = ls;

  // TODO: temporary fixed settings
  GridSettings gridSettings;
  gridSettings.gridSize = 5.0f;
  gridSettings.cellSize = 0.2f;

  collisionGrid.buildGrid(gridSettings);

  // initiate rng seed
  if(lSystem.seed == 0){
    srand(static_cast<unsigned>(time(0)) + timeOffset);
    timeOffset++;
  } else {
    srand(lSystem.seed);
  }

  return;
}

// generateTree: clears previous information and settings, sets new params and l-system, 
// and runs turtleGeneration
void TreeGenerator::generateTree(const TreeParameters params, const LSystem ls) {
  branches.clear();
  std::cout << "mesh cleared" << std::endl;
  mesh.clear();
  meshEdges.clear();
  collisionGrid.clearGrid();
  initiateTree(params, ls);
  loadTreeParameters("resources/" + ls.baseConfig);
  resolveLSystem(lSystem.passes);
  turtleGeneration(vec3(0.0f, 0.0f, -2.0f),
                     glm::normalize(quat(0.707f, 0.707f, 0.0f, 0.0f)));
}

// resolveLSystem : runs a given amount of passes of the L-System stored in
// the TreeGenerator class, returns the string
std::string TreeGenerator::resolveLSystem(const uint8_t passes) {
  std::string currentPass = lSystem.lState;
  std::string nextPass = "";
  maxDepth = 0;
  for (uint8_t pass = 0; pass < passes; pass++) {
    uint16_t branchDepth = 0;
    nextPass = "";
    // loop through each character and apply rule
    for (char token : currentPass) {
      // depth can be used for stochastic L-System generation
      if (token == '[' || token == '{') {
        branchDepth++;
        if (branchDepth > maxDepth) {
          maxDepth = branchDepth;
        }
      } else if (token == ']' || token == '}') {
        branchDepth--;
      }

      if (lSystem.ruleSet.count(token)) {
        Rule rule = lSystem.ruleSet[token];

        float totalChance = 0.0f;
        // normalize chances
        for (float chance : rule.afterChances) {
          totalChance += chance;
        }

        // iterate through chances
        std::string result = rule.afterList[0];
        float cumulativeChance = 0.0f;
        for (size_t chosenRule = 0; chosenRule < rule.afterList.size();
             chosenRule++) {
          // add up normalized chances
          cumulativeChance += rule.afterChances[chosenRule] / totalChance;
          if (RNG() - branchDepth * lSystem.depthBias < cumulativeChance) {
            result = rule.afterList[chosenRule];
            break;
          }
        }

        // append converted result
        nextPass += result;
      } else {
        // if no rule, simply add the token in its relative spot
        nextPass += token;
      }
    }

    currentPass = nextPass; // pass on new string to next loop
  }

  lSystem.lState = currentPass;
  return currentPass;
}

// RNG method so I can change rng process later - returns float between 0 and 1
float TreeGenerator::RNG() {
  return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

// testLSystem: basic test function for the tree generation system
void TreeGenerator::testLSystem() {
  std::map<char, Rule> rules;
  Rule r1;
  r1.before = 'a';
  r1.afterChances = {0.9f, 0.1f};
  r1.afterList = {"ab", ""};

  Rule r2;
  r2.before = 'b';
  r2.afterChances = {0.7f, 0.3f};
  r2.afterList = {"aa", "b"};

  rules['a'] = r1;
  rules['b'] = r2;
  lSystem.ruleSet = rules;

  lSystem.lState = "ab";

  std::string result = resolveLSystem(5);
  std::cout << "result: " << result << std::endl;
}

// turtleGeneration: traverse an L-System string to generate a mesh repreesnting a tree
// according to treeParameters and lSystem
void TreeGenerator::turtleGeneration(vec3 origin = vec3(0.0f),
                                     quat originRotation = quat(1.0f, 0.0f,
                                                                0.0f, 1.0f)) {

  // initiate first branch at depth 0, this is the trunk
  Branch firstBranch;
  firstBranch.globalOrigin = origin;
  firstBranch.globalOrientation= originRotation;
  firstBranch.depth = 0; 
  firstBranch.origin = origin;
  firstBranch.orientation = originRotation;
  firstBranch.parent = 0;
  branches.push_back(firstBranch);

  // initial turtle state and layers
  std::vector<TurtleState> currentTreeLayer;
  std::vector<TurtleState> nextTreeLayer;
  TurtleState initialState = {
    origin,                        
    originRotation,                   
    treeParameters.initialThickness,
    0,
    0,
    0
  };
  currentTreeLayer.push_back(initialState);

  // main loop - instruct turtle and switch layers for next loop
  while(currentTreeLayer.size() > 0){
    for(TurtleState& turtleState : currentTreeLayer){
      advanceTurtleState(turtleState, nextTreeLayer);
    }
    currentTreeLayer = nextTreeLayer;
    nextTreeLayer.clear();
  }
  
  // finish any post processing on the mesh
  polishMesh();
}

// advanceTurtleState: given a state of a turtle (contained in a tree layer), and a vector to add the next layer to
// progress the turtle forward according to the instruction from the L System
void TreeGenerator::advanceTurtleState(const TurtleState& state, std::vector<TurtleState>& nextLayer){
  if(state.lSystemIndex >= lSystem.lState.size()){
    return; //reached end
  }
  char instruction = lSystem.lState[state.lSystemIndex];

  if (instruction == 'F') {
    // move turtle forward with some randomized rotation

    // if a collision is detected, a different randomized rotation is tried until the branch finds a place
    // past COLLISION_RETRIES, the branch is abandoned and not added to the next layer
    for(uint8_t collisionAttempts = 0; collisionAttempts < COLLISION_RETRIES; collisionAttempts++){
      // temporary modified variables
      quat turtleRotation = state.turtleRotation;
      vec3 turtlePosition = state.turtlePosition;
      float turtleThickness = state.turtleThickness;

      // determine if section is heliotropic
      if (RNG() <= treeParameters.heliotropismChance) {
        turtleRotation = applyHeliotropism(turtleRotation, RNG() * treeParameters.heliotropismBendFactor);
      } else {
        // randomized non-heliotropic rotation
        turtleRotation =
            rotateBranchAbsolute(turtleRotation,
                        vec3((RNG() - 0.5f) * 2.0f * treeParameters.trunkBend,
                              (RNG() - 0.5f) * 2.0f * treeParameters.trunkTwist,
                              (RNG() - 0.5f) * 2.0f * treeParameters.trunkBend));
      }
      float branchLength =
          (2.0f - static_cast<float>(state.depth) / static_cast<float>(maxDepth)) *
          0.5f * treeParameters.branchLengthDepthFactor *
          treeParameters.branchLength;
      // make tree thinner the farther it grows
      turtleThickness = glm::clamp(
          turtleThickness * treeParameters.thicknessDecay,
          treeParameters.minThickness, treeParameters.initialThickness);

      // generate a temporary oriented bounding box to see if branch fits
      vec3 obbOrigin = turtlePosition + rotateVector(vec3(0.0f, 1.0f, 0.0f), turtleRotation) * branchLength * 0.5f;
      turtlePosition +=
          rotateVector(vec3(0.0f, 1.0f, 0.0f), turtleRotation) * branchLength;

      OBB sectionOBB;
      sectionOBB.origin = obbOrigin;
      sectionOBB.xDimension = glm::normalize(rotateVector(vec3(1.0f,0.0f,0.0f), turtleRotation));
      sectionOBB.yDimension = glm::normalize(rotateVector(vec3(0.0f,1.0f,0.0f), turtleRotation));
      sectionOBB.zDimension = glm::normalize(rotateVector(vec3(0.0f,0.0f,1.0f), turtleRotation));
      sectionOBB.dimensionSizes = vec3(turtleThickness, branchLength * 0.5f, turtleThickness);
      sectionOBB.branchIndex = state.branchIndex;

      // collisionGrid.addBox checks if branch can be added - if it can, it adds the branch to the collision grid
      // and returns true. else, it does not add the branch and returns false
      if(collisionGrid.addBox(sectionOBB, branches[state.branchIndex].parent)){ 
        // if the branch fits, we add its mesh to the vertex vector
        generateBranchMesh(state.branchIndex,
                        state.turtlePosition, turtleRotation, branchLength,
                        turtleThickness);
        
        // then we push the progressed turtle state to the next layer of the tree
        TurtleState progressedState = {
          turtlePosition,                        
          turtleRotation,                   
          turtleThickness,
          state.branchIndex,
          state.lSystemIndex+1,
          state.depth
        };
        nextLayer.push_back(progressedState);

        // DEBUG: if(collisionAttempts > 0 ) std::cout << static_cast<int>(collisionAttempts) << std::endl;
        break;
      }
    }

  } else if (instruction == '[') {
    // open bracket means a new branch is placed, and recursively drawn
    // first, place original turtle state back after branch, so have to find end of branch in string
    uint16_t branchDepth = 0;
    uint32_t checkStringIndex = state.lSystemIndex + 1;

    // find end of branch in L System string
    while(!(branchDepth == 0 && lSystem.lState[checkStringIndex] == ']')){
      if(lSystem.lState[checkStringIndex] == '['){
        branchDepth++;
      }
      else if(lSystem.lState[checkStringIndex] == ']'){
        branchDepth--;
      }

      checkStringIndex++;
    }
 
    float turtleThickness =
        treeParameters.branchOffRatio * (0.5 + RNG()) * state.turtleThickness;
    // clamp value: result is thickness of offshoot branch
    turtleThickness = glm::clamp(turtleThickness, treeParameters.minThickness,
                                 treeParameters.initialThickness);
    
    // tree continues from original position, after branch ends in the L System string
    TurtleState repeatedState = {
      state.turtlePosition,
      state.turtleRotation,
      sqrt(state.turtleThickness * state.turtleThickness - turtleThickness * turtleThickness), // thickness equation
      state.branchIndex,
      checkStringIndex + 1,
      state.depth
    };
    nextLayer.push_back(repeatedState);

    // branch off by rotating turtle away, in local rotation
    float eulerX = (RNG() - 0.5f) * 2.0f * treeParameters.branchBend;
    if (abs(eulerX) <= treeParameters.branchMinBend) {
      eulerX = treeParameters.branchMinBend * glm::sign(eulerX);
    }
    float eulerY = (RNG() - 0.5f) * 2.0f * treeParameters.branchTwist;
    float eulerZ = (RNG() - 0.5f) * 2.0f * treeParameters.branchBend;
    if (abs(eulerZ) <= treeParameters.branchMinBend) {
      eulerZ = treeParameters.branchMinBend * glm::sign(eulerZ);
    }
    quat turtleRotation = rotateBranchAbsolute(state.turtleRotation, vec3(eulerX, eulerY, eulerZ));

    // generate and add branch to branches
    Branch addBranch = generateSingleBranch(state.turtlePosition, state.turtleRotation, state.branchIndex, state.depth + 1);
    branches.push_back(addBranch);

    // add offshoot branch to next layer
    TurtleState branchState = {
      state.turtlePosition,
      turtleRotation,
      turtleThickness,
      static_cast<uint32_t>(branches.size() - 1),
      checkStringIndex + 1,
      static_cast<uint16_t>(state.depth + 1)
    };
    nextLayer.push_back(branchState);

  } else if (instruction > '0' && instruction <= '9'){
    // if int is found - splitting instruction started
    uint8_t splitCount = instruction - '0';
    float splitOffset = RNG() * 2.0f * glm::pi<float>();
    float splitAngle = 2.0f * glm::pi<float>() / splitCount;
    float baseThickness = state.turtleThickness / sqrt((float) splitCount);
    uint32_t lSystemIndex = state.lSystemIndex + 2;

    // create as many splits as is specified according to the instruction
    for(uint8_t split = 0; split < splitCount; split++){
      // determine thickness, bend, twist
      float thicknessRNG = treeParameters.minSplitOffThicknessFactor+ (treeParameters.maxSplitOffThicknessFactor - treeParameters.minSplitOffThicknessFactor) * RNG();
      float currentThickness = thicknessRNG * baseThickness;

      float twist = splitOffset + split * splitAngle + (RNG() - 0.5f) * 2.0f * treeParameters.splitOffTwist;
      float bend = glm::clamp(RNG() * treeParameters.splitOffBend, treeParameters.minSplitOffBend, treeParameters.splitOffBend);

      // rotation math
      vec3 rotationAmount = vec3(
        0.0f,
        twist,
        0.0f
      );
      quat interRotation = rotateBranchAbsolute(state.turtleRotation, rotationAmount);
      rotationAmount = vec3(
        bend,
        0.0f,
        0.0f
      );
      quat turtleRotation = rotateBranchAbsolute(interRotation, rotationAmount);
      
      // offset branches in their directions
      const vec3 yAxis = rotateVector(vec3(0.0f, state.turtleThickness * 0.5, 0.0f), turtleRotation);
      const vec3 xAxis = glm::normalize(rotateVector(vec3(1.0f, 0.0f, 0.0f), state.turtleRotation));
      const vec3 zAxis = glm::normalize(rotateVector(vec3(0.0f, 0.0f, 1.0f), state.turtleRotation));
      vec3 xProj = glm::dot(yAxis, xAxis) * xAxis;
      vec3 zProj = glm::dot(yAxis, zAxis) * zAxis;
      vec3 turtlePosition = state.turtlePosition + xProj + zProj;

      // create new branch and add to branches
      Branch addBranch = generateSingleBranch(turtlePosition, turtleRotation, state.branchIndex, state.depth + 1);
      branches.push_back(addBranch);

      // generate split-off branch in this loop and add it to the next layer of the tree
      TurtleState splitState = {
        turtlePosition,
        turtleRotation,
        currentThickness,
        static_cast<uint32_t>(branches.size() - 1),
        lSystemIndex,
        static_cast<uint16_t>(state.depth + 1)
      };
      nextLayer.push_back(splitState);

      // find end of this split-off branch, set next index to the beginning of the next
      uint16_t branchDepth = 0;
      while(!(branchDepth == 0 && lSystem.lState[lSystemIndex] == '}')){
        if(lSystem.lState[lSystemIndex] == '{'){
          branchDepth++;
        }
        else if(lSystem.lState[lSystemIndex] == '}'){
          branchDepth--;
        }

        lSystemIndex++;
      }
      // skip the "}{" separating the branches
      lSystemIndex += 2;
    }
  }
}

// generateSingleBranch: generates a branch representing a section of the tree
// branches will be used to calculate physics later
// stores both global and local coordinates
Branch TreeGenerator::generateSingleBranch(const vec3& origin, const quat& originRotation, 
                                           uint32_t parentIndex, uint16_t depth) {
  Branch newBranch;
  newBranch.globalOrigin = origin;
  newBranch.globalOrientation= originRotation; // TODO: apply some randomization here
  newBranch.depth = depth;
  newBranch.parent = parentIndex;
  newBranch.origin = origin - branches[parentIndex].origin;
  newBranch.orientation = originRotation - branches[parentIndex].orientation;

  return newBranch;
}

// rotateBranchAbsolute: rotate a quat rotation representing a branch's orientation, done with local vectors
// rotates a given rotation by an amount in each x,y,z dimensions
glm::quat TreeGenerator::rotateBranchAbsolute(const quat &rotation,
                                              const vec3 amount) {
  vec3 rightAxis = vec3(1.0f, 0.0f, 0.0f);
  quat runningRotation = glm::rotate(rotation, amount.x, rightAxis);

  vec3 forwardAxis = vec3(0.0f, 1.0f, 0.0f);
  runningRotation = glm::rotate(runningRotation, amount.y, forwardAxis);

  vec3 upAxis = vec3(0.0f, 0.0f, 1.0f);
  runningRotation = glm::rotate(runningRotation, amount.z, upAxis);

  return runningRotation;
}

// rotateVector: rotate a vector by a given quat rotation
glm::vec3 TreeGenerator::rotateVector(const vec3 &vector,
                                      const quat &rotation) {
  vec3 rotVec = vec3(rotation.x, rotation.y, rotation.z);
  return vector + 2.0f * glm::cross(rotVec, glm::cross(rotVec, vector) +
                                                rotation.w * vector);
}

void TreeGenerator::generateBranchMesh(const uint32_t branchIndex,
                                       const vec3 &origin, const quat &rotation,
                                       const float sectionLength,
                                       const float thickness) {
  float verticalSliceSize = 0.05f;
  uint8_t minHorizontalSampleCount = 4;
  uint8_t baseHorizontalSampleCount = 32;
  uint8_t horizontalSampleCount = minHorizontalSampleCount + static_cast<uint8_t>((thickness / treeParameters.initialThickness) * (baseHorizontalSampleCount - minHorizontalSampleCount));
  float horizontalSliceSize = 2.0f * glm::pi<float>() / static_cast<float>(horizontalSampleCount);

  std::vector<uint32_t> closeVertices;
  std::vector<vec3> vertices;
  std::vector<vec3> normals;

  //  get axes
  const vec3 yAxis = glm::normalize(rotateVector(vec3(0.0f, 1.0f, 0.0f), rotation));
  const vec3 xAxis = glm::normalize(rotateVector(vec3(1.0f, 0.0f, 0.0f), rotation));

  uint16_t verticalCount = 0;
  float cumulativeVertical = 0.0f;
  bool reachedEnd = false;

  while(cumulativeVertical <= sectionLength){
    float horizontalOffset = RNG() * horizontalSliceSize; // randomize starting offset
    vec3 yPos = cumulativeVertical * yAxis;

    for(uint8_t hSample = 0; hSample < horizontalSampleCount; hSample++){
      float rotationAmount = hSample * horizontalSliceSize + (RNG() + 0.2f) * 0.83f * horizontalSliceSize + horizontalOffset;
      //quat sampleRotation = glm::rotate(rotation, rotationAmount, yAxis);
      //vec3 rotatedX = glm::normalize(rotateVector(vec3(1.0f, 0.0f, 0.0f), sampleRotation)) *
      //         (thickness + (thickness * (RNG() - 0.5f) * 0.2f));

      vec3 rotatedX = xAxis + glm::sin(rotationAmount) * glm::cross(yAxis, xAxis) + 
                              (1.0f - glm::cos(rotationAmount)) * glm::cross(yAxis, glm::cross(yAxis, xAxis));
      rotatedX = glm::normalize(rotatedX) * (thickness + thickness * ((RNG() - 0.5f) * 0.05f));

      vec3 sample = origin + 
        yPos +  
        ((RNG() - 0.5f) * 0.25f * yAxis * verticalSliceSize) + 
        rotatedX;
      vertices.push_back(sample);
      vec3 normal = sample - (origin + yPos);
      normals.push_back(normal);
    }
    
    cumulativeVertical += (RNG() - 0.5f) * 0.25f * verticalSliceSize + verticalSliceSize;
    if(cumulativeVertical > sectionLength && !reachedEnd){
      cumulativeVertical = sectionLength;
      reachedEnd = true;
    }
    verticalCount++;
  }

  std::vector<uint32_t> previousLayer;
  // connect
  if(branchIndex < meshEdges.size() && meshEdges[branchIndex].size() > 0){
    previousLayer = meshEdges[branchIndex];
  }

  if(previousLayer.size() > 0){
    float minimumDist = 10000.0f;
    uint16_t minDistVertex = 0;
    for(uint16_t connectionIndex = 0; connectionIndex < horizontalSampleCount * 2 && connectionIndex < vertices.size(); connectionIndex++){
      float currentDist = glm::length(mesh[previousLayer[0]].position - vertices[connectionIndex]);
      if(currentDist < minimumDist){
        minimumDist = currentDist;
        minDistVertex = connectionIndex; 
      }
    }
    uint16_t startingEdge = minDistVertex;

    uint16_t currentTopConnection = (startingEdge + 1) % horizontalSampleCount;
    uint16_t currentBottomConnection = 0;
    // initial square - doing it this way was for debugging at first but will keep this for loop repition purposes
    mesh.push_back(TreeMeshVertex{mesh[previousLayer[0]].position,0.0f,mesh[previousLayer[0]].normal,0.0f,branchIndex});
    mesh.push_back(TreeMeshVertex{vertices[startingEdge],0.0f,normals[startingEdge],0.0f,branchIndex});
    mesh.push_back(TreeMeshVertex{mesh[previousLayer[1]].position,0.0f,mesh[previousLayer[1]].normal,0.0f,branchIndex});

    mesh.push_back(TreeMeshVertex{vertices[startingEdge],0.0f,normals[startingEdge],0.0f,branchIndex});
    mesh.push_back(TreeMeshVertex{vertices[currentTopConnection],0.0f,normals[currentTopConnection],0.0f,branchIndex});
    mesh.push_back(TreeMeshVertex{mesh[previousLayer[0]].position,0.0f,mesh[previousLayer[0]].normal,0.0f,branchIndex});

    while(currentTopConnection != startingEdge && currentBottomConnection != 1){
      uint16_t topWrap = (currentTopConnection + 1) % horizontalSampleCount;
      uint16_t botWrap = (currentBottomConnection + static_cast<uint16_t>(previousLayer.size() - 1)) % previousLayer.size();

      mesh.push_back(TreeMeshVertex{mesh[previousLayer[botWrap]].position,0.0f,mesh[previousLayer[botWrap]].normal,0.0f,branchIndex});
      mesh.push_back(TreeMeshVertex{vertices[currentTopConnection],0.0f,normals[currentTopConnection],0.0f,branchIndex});
      mesh.push_back(TreeMeshVertex{mesh[previousLayer[currentBottomConnection]].position,0.0f,mesh[previousLayer[currentBottomConnection]].normal,0.0f,branchIndex});

      mesh.push_back(TreeMeshVertex{vertices[currentTopConnection],0.0f,normals[currentTopConnection],0.0f,branchIndex});
      mesh.push_back(TreeMeshVertex{vertices[topWrap],0.0f,normals[topWrap],0.0f,branchIndex});
      mesh.push_back(TreeMeshVertex{mesh[previousLayer[botWrap]].position,0.0f,mesh[previousLayer[botWrap]].normal,0.0f,branchIndex});

      currentTopConnection = topWrap;
      currentBottomConnection = botWrap;
    }
    while(currentBottomConnection != 1){
      // plug up extra vertex gaps
      uint16_t botWrap = (currentBottomConnection + static_cast<uint16_t>(previousLayer.size() - 1)) % previousLayer.size();

      mesh.push_back(TreeMeshVertex{mesh[previousLayer[botWrap]].position,0.0f,mesh[previousLayer[botWrap]].normal,0.0f,branchIndex});
      mesh.push_back(TreeMeshVertex{vertices[currentTopConnection],0.0f,normals[currentTopConnection],0.0f,branchIndex});
      mesh.push_back(TreeMeshVertex{mesh[previousLayer[currentBottomConnection]].position,0.0f,mesh[previousLayer[currentBottomConnection]].normal,0.0f,branchIndex});

      currentBottomConnection = botWrap;
    }
  }

  // normal trunk/branch
  for(uint16_t layer = 0; layer < verticalCount - 1; layer++){
    for(uint8_t hSample = 0; hSample < horizontalSampleCount; hSample++){
      uint8_t hSampleWrap = (hSample + 1) % horizontalSampleCount;
      mesh.push_back(TreeMeshVertex{
        vertices[layer * horizontalSampleCount + hSample],
        0.0f,
        normals[layer * horizontalSampleCount + hSample],
        0.0f,
        branchIndex
      });
      mesh.push_back(TreeMeshVertex{
        vertices[(layer + 1) * horizontalSampleCount + hSample],
        0.0f,
        normals[(layer + 1) * horizontalSampleCount + hSample],
        0.0f,
        branchIndex
      });
      mesh.push_back(TreeMeshVertex{
        vertices[layer * horizontalSampleCount + hSampleWrap],
        0.0f,
        normals[layer * horizontalSampleCount + hSampleWrap],
        0.0f,
        branchIndex
      });

      mesh.push_back(TreeMeshVertex{
        vertices[(layer + 1) * horizontalSampleCount + hSample],
        0.0f,
        normals[(layer + 1) * horizontalSampleCount + hSample],
        0.0f,
        branchIndex
      });
      mesh.push_back(TreeMeshVertex{
        vertices[(layer + 1) * horizontalSampleCount + hSampleWrap],
        0.0f,
        normals[(layer + 1) * horizontalSampleCount + hSampleWrap],
        0.0f,
        branchIndex
      });
      mesh.push_back(TreeMeshVertex{
        vertices[layer * horizontalSampleCount + hSampleWrap],
        0.0f,
        normals[layer * horizontalSampleCount + hSampleWrap],
        0.0f,
        branchIndex
      });
    }
  }

  while(branchIndex >= meshEdges.size()){
    meshEdges.push_back(std::vector<uint32_t>()); 
  }

  std::vector<uint32_t> newBranchEdge;
  for(uint8_t layerVertex = 0; layerVertex < horizontalSampleCount; layerVertex++){
    newBranchEdge.push_back(static_cast<uint32_t>(mesh.size()) - 5 - 6 * (layerVertex));
  }
  meshEdges[branchIndex] = newBranchEdge;

}

// generateSquareBranchMesh: generate a prism mesh given a start point, rotation, length, and thickness
// starts centered in x, z, but at the beginning of the y direction
void TreeGenerator::generateSquareBranchMesh(const uint32_t branchIndex,
                                       const vec3 &origin, const quat &rotation,
                                       const float sectionLength,
                                       const float thickness) {
  Branch branch = branches[branchIndex];

  //  get axes
  vec3 xAxis = glm::normalize(rotateVector(vec3(1.0f, 0.0f, 0.0f), rotation)) *
               thickness;
  vec3 yAxis = glm::normalize(rotateVector(vec3(0.0f, 1.0f, 0.0f), rotation)) *
               sectionLength;
  vec3 zAxis = glm::normalize(rotateVector(vec3(0.0f, 0.0f, 1.0f), rotation)) *
               thickness;

  // first face in x direction
  mesh.push_back(TreeMeshVertex{origin + xAxis - zAxis, 0.0f, xAxis, 0.0f});
  mesh.push_back(TreeMeshVertex{origin + xAxis + zAxis, 0.0f, xAxis, 0.0f});
  mesh.push_back(
      TreeMeshVertex{origin + xAxis + yAxis + zAxis, 0.0f, xAxis, 0.0f});

  mesh.push_back(
      TreeMeshVertex{origin + xAxis + yAxis + zAxis, 0.0f, xAxis, 0.0f});
  mesh.push_back(
      TreeMeshVertex{origin + xAxis + yAxis - zAxis, 0.0f, xAxis, 0.0f});
  mesh.push_back(TreeMeshVertex{origin + xAxis - zAxis, 0.0f, xAxis, 0.0f});

  // second face in x direction
  mesh.push_back(TreeMeshVertex{origin + -xAxis + zAxis, 0.0f, -xAxis, 0.0f});
  mesh.push_back(TreeMeshVertex{origin + -xAxis - zAxis, 0.0f, -xAxis, 0.0f});
  mesh.push_back(
      TreeMeshVertex{origin + -xAxis + yAxis - zAxis, 0.0f, -xAxis, 0.0f});

  mesh.push_back(
      TreeMeshVertex{origin + -xAxis + yAxis - zAxis, 0.0f, -xAxis, 0.0f});
  mesh.push_back(
      TreeMeshVertex{origin + -xAxis + yAxis + zAxis, 0.0f, -xAxis, 0.0f});
  mesh.push_back(TreeMeshVertex{origin + -xAxis + zAxis, 0.0f, -xAxis, 0.0f});

  // first face in z direction
  mesh.push_back(TreeMeshVertex{origin + xAxis + zAxis, 0.0f, zAxis, 0.0f});
  mesh.push_back(TreeMeshVertex{origin + -xAxis + zAxis, 0.0f, zAxis, 0.0f});
  mesh.push_back(
      TreeMeshVertex{origin + -xAxis + yAxis + zAxis, 0.0f, zAxis, 0.0f});

  mesh.push_back(
      TreeMeshVertex{origin + -xAxis + yAxis + zAxis, 0.0f, zAxis, 0.0f});
  mesh.push_back(
      TreeMeshVertex{origin + xAxis + yAxis + zAxis, 0.0f, zAxis, 0.0f});
  mesh.push_back(TreeMeshVertex{origin + xAxis + zAxis, 0.0f, zAxis, 0.0f});

  // first face in z direction
  mesh.push_back(TreeMeshVertex{origin + -xAxis - zAxis, 0.0f, -zAxis, 0.0f});
  mesh.push_back(TreeMeshVertex{origin + xAxis - zAxis, 0.0f, -zAxis, 0.0f});
  mesh.push_back(
      TreeMeshVertex{origin + xAxis + yAxis - zAxis, 0.0f, -zAxis, 0.0f});

  mesh.push_back(
      TreeMeshVertex{origin + xAxis + yAxis - zAxis, 0.0f, -zAxis, 0.0f});
  mesh.push_back(
      TreeMeshVertex{origin + -xAxis + yAxis - zAxis, 0.0f, -zAxis, 0.0f});
  mesh.push_back(TreeMeshVertex{origin + -xAxis - zAxis, 0.0f, -zAxis, 0.0f});

  // for now we will skip the top and bottom faces
}

void TreeGenerator::polishMesh(){
  for(uint32_t branch = 0; branch < meshEdges.size(); branch++){
    std::vector<uint32_t> branchEdge = meshEdges[branch];
    if(branchEdge.size() > 0){
      vec3 center = vec3(0.0f);
      std::vector<vec3> prevVerts;
      std::vector<vec3> prevNorms;
      for(uint32_t meshIndex : branchEdge){
        center += mesh[meshIndex].position;
        prevVerts.push_back(mesh[meshIndex].position);
        prevNorms.push_back(mesh[meshIndex].normal);
      }
      center /= branchEdge.size();
      vec3 radius1 = mesh[branchEdge[0]].position - center;
      vec3 radius2 = mesh[branchEdge[branchEdge.size() - 1]].position - center;
      float radius = glm::length(radius1);
      vec3 upDir = glm::normalize(glm::cross(radius1, radius2));
      vec3 top = center + upDir * radius;

      std::vector<vec3> layerVerts;
      std::vector<vec3> layerNorms;
      for(uint8_t layer = 0; layer < 3; layer++){
        for(uint32_t meshIndex : branchEdge){
          float sphereX = (3 - layer) * 0.25f * radius;
          float sphereY = sqrt(radius * radius - sphereX * sphereX);

          layerVerts.push_back(center + 
                              upDir * sphereY +
                              glm::normalize(mesh[meshIndex].position - center) * sphereX);
          layerNorms.push_back(layerVerts[layerVerts.size() - 1] - center);
        }  

        for(uint8_t indexInLayer = 0; indexInLayer < prevVerts.size(); indexInLayer++){
          uint8_t indWrap = (indexInLayer + 1) % prevVerts.size();

          mesh.push_back(TreeMeshVertex{
            prevVerts[indexInLayer],
            0.0f,
            prevNorms[indexInLayer],
            0.0f,
            branch
          });
          mesh.push_back(TreeMeshVertex{
            layerVerts[indexInLayer],
            0.0f,
            layerNorms[indexInLayer],
            0.0f,
            branch
          });
          mesh.push_back(TreeMeshVertex{
            prevVerts[indWrap],
            0.0f,
            prevNorms[indWrap],
            0.0f,
            branch
          });

          mesh.push_back(TreeMeshVertex{
            layerVerts[indexInLayer],
            0.0f,
            layerNorms[indexInLayer],
            0.0f,
            branch
          });
          mesh.push_back(TreeMeshVertex{
            layerVerts[indWrap],
            0.0f,
            layerNorms[indWrap],
            0.0f,
            branch
          });
          mesh.push_back(TreeMeshVertex{
            prevVerts[indWrap],
            0.0f,
            prevNorms[indWrap],
            0.0f,
            branch
          });
        }
        
        prevNorms = layerNorms;
        prevVerts = layerVerts;
        layerNorms.clear();
        layerVerts.clear();
      }

      for(uint8_t indexInLayer = 0; indexInLayer < prevVerts.size(); indexInLayer++){
        uint8_t indWrap = (indexInLayer + 1) % prevVerts.size();

          mesh.push_back(TreeMeshVertex{
            prevVerts[indexInLayer],
            0.0f,
            prevNorms[indexInLayer],
            0.0f,
            branch
          });
          mesh.push_back(TreeMeshVertex{
            center + upDir * radius,
            0.0f,
            upDir,
            0.0f,
            branch
          });
          mesh.push_back(TreeMeshVertex{
            prevVerts[indWrap],
            0.0f,
            prevNorms[indWrap],
            0.0f,
            branch
          });
      }
    }
  }
}

// loadTreeParameters: loads a json file containing the ruleset for an L System 
void TreeGenerator::loadTreeParameters(const std::string &fileName) {
  std::cout << fileName << std::endl;
  std::ifstream inputJson(fileName);
  json jsonData = json::parse(inputJson);

  lSystem.lState = jsonData["lState"];
  for (auto &ruleData : jsonData["ruleSet"].items()) {
    Rule rule;
    rule.before = ruleData.key()[0];

    for (auto &ruleChance : ruleData.value().items()) {
      rule.afterList.push_back(ruleChance.key());
      rule.afterChances.push_back(static_cast<float>(ruleChance.value()));
    }

    lSystem.ruleSet[rule.before] = rule;
  }
}

// applyHeliotropism: rotate a rotation upwards by a given amount
glm::quat TreeGenerator::applyHeliotropism(const quat& rotation, const float amount){
  quat upwards = quat(0.707f, 0.707f, 0.0f, 0.0f);
  float angle = glm::acos(glm::dot(rotation, upwards));
  return glm::slerp(rotation, upwards, amount/angle);
}