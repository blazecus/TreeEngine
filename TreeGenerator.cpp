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

void TreeGenerator::generateTree(const TreeParameters params, const LSystem ls) {
  branches.clear();
  mesh.clear();
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
    // std::cout << "pass: " << pass << " " << currentPass << std::endl;
    // std::cout << "------------" << std::endl;
  }

  lSystem.lState = currentPass;
  //std::cout << lSystem.lState << std::endl;
  return currentPass;
}

// RNG method so I can change rng process later - returns float between 0 and 1
float TreeGenerator::RNG() {
  return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

// basic test function for the tree generation system
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

// note for later - branches need local coordinates. relative positions can be
// calculated as forces are, then vertex positions have to be calculated
// backwards
void TreeGenerator::turtleGeneration(vec3 origin = vec3(0.0f),
                                     quat originRotation = quat(1.0f, 0.0f,
                                                                0.0f, 1.0f)) {

  // initiate first branch at depth 0, this is the trunk
  Branch firstBranch;
  firstBranch.globalOrigin = origin;
  firstBranch.globalOrientation= originRotation; // TODO: apply some randomization here
  firstBranch.depth = 0; 
  firstBranch.origin = origin;
  firstBranch.orientation = originRotation;
  firstBranch.parent = 0;

  branches.push_back(firstBranch);
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

  // these stacks keep track of where we
  /*std::vector<uint32_t> branchStack{0};
  std::vector<vec3> positionStack{origin};
  std::vector<quat> rotationStack{originRotation};
  std::vector<float> thicknessStack{treeParameters.initialThickness};
  

  // need to keep track of turtle state as it can move without saving to the
  // stack
  vec3 turtlePosition = origin;
  quat turtleRotation = originRotation;
  float turtleThickness = treeParameters.initialThickness;

  for (char instruction : lSystem.lState) {
    instructTurtle(instruction, branchStack, positionStack, rotationStack,
                   thicknessStack, depth, turtlePosition, turtleRotation,
                   turtleThickness);
  }*/
  //uint32_t layerCount = 0;
  while(currentTreeLayer.size() > 0){
    /*
    std::cout << "Layer: " << layerCount << std::endl;
    std::cout << "Current layer size: " << currentTreeLayer.size() << std::endl;
    std::cout << "-------------------" << std::endl;
    layerCount++;
    */
    for(TurtleState& turtleState : currentTreeLayer){
      advanceTurtleState(turtleState, nextTreeLayer);
    }
    currentTreeLayer = nextTreeLayer;
    nextTreeLayer.clear();
  }

}

/*void TreeGenerator::instructTurtle(
    char instruction, std::vector<uint32_t> &branchStack,
    std::vector<vec3> &positionStack, std::vector<quat> &rotationStack,
    std::vector<float> &thicknessStack, uint16_t &depth, vec3 &turtlePosition,
    quat &turtleRotation, float &turtleThickness) {
*/
void TreeGenerator::advanceTurtleState(const TurtleState& state, std::vector<TurtleState>& nextLayer){
  // retrieve the branch we're currently traversing (top level branch in branch
  // stack)
  if(state.lSystemIndex >= lSystem.lState.size()){
    return; //reached end
  }
  char instruction = lSystem.lState[state.lSystemIndex];

  if (instruction == 'F') {
    // move turtle forward with some rotation
    // apply rotation first
    for(uint8_t collisionAttempts = 0; collisionAttempts < COLLISION_RETRIES; collisionAttempts++){
      quat turtleRotation = state.turtleRotation;
      vec3 turtlePosition = state.turtlePosition;
      float turtleThickness = state.turtleThickness;

      if (RNG() <= treeParameters.heliotropismChance) {
        turtleRotation = applyHeliotropism(turtleRotation, RNG() * treeParameters.heliotropismBendFactor);
      } else {
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

      if(collisionGrid.addBox(sectionOBB, branches[state.branchIndex].parent)){ 
        generateBranchMesh(state.branchIndex,
                        state.turtlePosition, turtleRotation, branchLength,
                        turtleThickness);
        TurtleState progressedState = {
          turtlePosition,                        
          turtleRotation,                   
          turtleThickness,
          state.branchIndex,
          state.lSystemIndex+1,
          state.depth
        };
        
        nextLayer.push_back(progressedState);
        //if(collisionAttempts > 0 ) std::cout << static_cast<int>(collisionAttempts) << std::endl;
        break;
      }
    }

  } else if (instruction == '[') {
    // open bracket means a new branch is placed, and recursively drawn
    // first, place original turtle state back after branch, so have to find end of branch in string
    uint16_t branchDepth = 0;
    uint32_t checkStringIndex = state.lSystemIndex + 1;
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
    // clamp value
    turtleThickness = glm::clamp(turtleThickness, treeParameters.minThickness,
                                 treeParameters.initialThickness);

    TurtleState repeatedState = {
      state.turtlePosition,
      state.turtleRotation,
      sqrt(state.turtleThickness * state.turtleThickness - turtleThickness * turtleThickness),
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

    // place origin of next branch at local position, not global position
    /*Branch addBranch = generateSingleBranch(
        turtlePosition - positionStack[positionStack.size() - 2],
        turtleRotation - rotationStack[rotationStack.size() - 2], depth);
    addBranch.parent = branchStack[branchStack.size() - 2];
    */
   Branch addBranch = generateSingleBranch(state.turtlePosition, state.turtleRotation, state.branchIndex, state.depth + 1);

    // add branch to branches
    branches.push_back(addBranch);
    TurtleState branchState = {
      state.turtlePosition,
      turtleRotation,
      turtleThickness,
      static_cast<uint32_t>(branches.size() - 1),
      checkStringIndex + 1,
      static_cast<uint16_t>(state.depth + 1)
    };

    nextLayer.push_back(branchState);
    
  } else if (instruction == ']' || instruction == '}') {
    // closed bracket means branch is ended, and turtle returns to the beginning
    // of the branch
    /*depth--;

    turtlePosition = positionStack[positionStack.size() - 1];
    turtleRotation = rotationStack[rotationStack.size() - 1];
    turtleThickness = thicknessStack[thicknessStack.size() - 1];

    // get rid of stored branch
    branchStack.pop_back();
    positionStack.pop_back();
    rotationStack.pop_back();
    thicknessStack.pop_back();*/
  } else if (instruction > '0' && instruction <= '9'){
    // if int is found - splitting instruction started
    uint8_t splitCount = instruction - '0';
    float splitOffset = RNG() * 2.0f * glm::pi<float>();
    float splitAngle = 2.0f * glm::pi<float>() / splitCount;

    float baseThickness = state.turtleThickness / sqrt((float) splitCount);

    /*
    vec3 parentPosition = positionStack[positionStack.size() - 1];
    quat parentRotation = rotationStack[rotationStack.size() - 1];
    uint32_t parentBranch = branchStack[branchStack.size() - 1];
    */
    uint32_t lSystemIndex = state.lSystemIndex + 2;

    for(uint8_t split = 0; split < splitCount; split++){
      float thicknessRNG = treeParameters.minSplitOffThicknessFactor+ (treeParameters.maxSplitOffThicknessFactor - treeParameters.minSplitOffThicknessFactor) * RNG();
      float currentThickness = thicknessRNG * baseThickness;

      float twist = splitOffset + split * splitAngle + (RNG() - 0.5f) * 2.0f * treeParameters.splitOffTwist;
      float bend = glm::clamp(RNG() * treeParameters.splitOffBend, treeParameters.minSplitOffBend, treeParameters.splitOffBend);

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

      /*
      thicknessStack.push_back(currentThickness);
      positionStack.push_back(turtlePosition);
      rotationStack.push_back(rotateBranchAbsolute(interRotation, rotationAmount));
      branchStack.push_back(static_cast<uint32_t>(branches.size()));   

      // place origin of next branch at local position, not global position
      Branch addBranch = generateSingleBranch(
          turtlePosition - parentPosition,
          turtleRotation - parentRotation, depth);
      addBranch.parent = parentBranch;

      // add branch to branches
      branches.push_back(addBranch);
      */
      quat turtleRotation = rotateBranchAbsolute(interRotation, rotationAmount);
      Branch addBranch = generateSingleBranch(state.turtlePosition, turtleRotation, state.branchIndex, state.depth + 1);

      // add branch to branches
      branches.push_back(addBranch);
      TurtleState splitState = {
        state.turtlePosition,
        turtleRotation,
        currentThickness,
        static_cast<uint32_t>(branches.size() - 1),
        lSystemIndex,
        static_cast<uint16_t>(state.depth + 1)
      };

      nextLayer.push_back(splitState);
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
      lSystemIndex += 2;
    }
  } else if (instruction == '{'){
    /*
    depth ++;
    turtlePosition = positionStack[positionStack.size()-1];
    turtleRotation = rotationStack[rotationStack.size()-1];
    turtleThickness = thicknessStack[thicknessStack.size()-1];
    */
  }
}

Branch TreeGenerator::generateSingleBranch(const vec3& origin, const quat& originRotation, 
                                           uint32_t parentIndex, uint16_t depth) {
  Branch newBranch;
  newBranch.globalOrigin = origin;
  newBranch.globalOrientation= originRotation; // TODO: apply some randomization here
  newBranch.depth = depth;
  newBranch.parent = parentIndex;
  newBranch.origin = origin - branches[parentIndex].origin;
  newBranch.orientation = originRotation - branches[parentIndex].orientation;

  // depth should be the main factor affecting randomness

  // apply random rotation according to depth
  return newBranch;
}

glm::quat TreeGenerator::rotateBranch(const quat &rotation, const vec3 amount) {
  vec3 rightAxis = rotateVector(vec3(1.0f, 0.0f, 0.0f), rotation);
  quat runningRotation = glm::rotate(rotation, amount.x, rightAxis);

  vec3 forwardAxis = rotateVector(vec3(0.0f, 1.0f, 0.0f), rotation);
  runningRotation = glm::rotate(runningRotation, amount.y, forwardAxis);

  vec3 upAxis = rotateVector(vec3(0.0f, 0.0f, 1.0f), rotation);
  runningRotation = glm::rotate(runningRotation, amount.z, upAxis);

  return runningRotation;
}

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
  // std::cout << mesh[mesh.size() - 1].position.x << std::endl;
  // std::cout << mesh[mesh.size() - 1].position.y << std::endl;
  // std::cout << mesh[mesh.size() - 1].position.z << std::endl;
  // std::cout << "--" << std::endl;

  // for now we will skip the top and bottom faces
}

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

glm::quat TreeGenerator::applyHeliotropism(const quat& rotation, const float amount){
  quat upwards = quat(0.707f, 0.707f, 0.0f, 0.0f);
  float angle = glm::acos(glm::dot(rotation, upwards));
  return glm::slerp(rotation, upwards, amount/angle);
}