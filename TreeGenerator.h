#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <json.hpp>
#include <map>
#include <string>
#include <vector>
#include "OccupancyGrid.h"

class TreeGenerator {
public:

  static const uint8_t COLLISION_RETRIES = 20;

  // (Just aliases to make notations lighter)
  using vec3 = glm::vec3;
  using vec2 = glm::vec2;
  using mat3x3 = glm::mat3x3;
  using quat = glm::quat;
  using OBB = OccupancyGrid::OBB;

  // represents one rule : takes in a single char, and turns it into a string
  // based on a list of chances chances are interpreted as "weights", and
  // normalized by the sum of all the weights
  struct Rule {
    char before;                        // character that is transformed by rule
    std::vector<std::string> afterList; // indexed list of possible results
    std::vector<float> afterChances; // indexed list of chances for each result
  };

  // represents a branch in the tree - these will be passed into the compute
  // shader
  struct Branch {
    vec3 origin;
    quat orientation;
    vec3 globalOrigin;
    quat globalOrientation;

    float length;

    vec3 linearForce;
    mat3x3 angularForce;

    float thickness;
    float pliancy;
    float maxBend;
    float mass;
    uint16_t depth;

    uint32_t parent;
  };

  struct TurtleState{
    vec3 turtlePosition;
    quat turtleRotation;
    float turtleThickness;
    uint32_t branchIndex;
    uint32_t lSystemIndex;
    uint16_t depth;
  };

  // Mesh buffer input
  struct TreeMeshVertex {
    vec3 position = vec3(0.0f, 0.0f, 0.0f);
    float garbage1; // 16 byte alignment
    vec3 normal = vec3(0.0f, 1.0f, 0.0f);
    float garbage2; // 16 byte alignment
  };

  // parameters that determine tree generation (branch and mesh representations)
  struct TreeParameters {
    float trunkTwist = 0.2f;
    float trunkBend = 0.4f;

    float branchTwist = 0.8f;
    float branchMinBend = 0.4f;
    float branchBend = 1.2f;

    float branchLength = 0.2f;
    float branchLengthDepthFactor = 2.0f;

    float heliotropismChance = 0.6f;
    float heliotropismBendFactor = 0.4f;
    float initialThickness = 0.3f;
    float minThickness = 0.006f;
    float thicknessDecay = 0.99f;
    float branchOffRatio = 0.4f;

    float splitOffBend = 1.3f;
    float minSplitOffBend = 0.5f;
    float splitOffTwist = 0.1f;
    float maxSplitOffThicknessFactor = 1.2f;
    float minSplitOffThicknessFactor = 1.15f;
  };

  // parameters and variables that govern L-System generation
  struct LSystem {
    std::string lState;
    std::map<char, Rule> ruleSet;
    float depthBias = 0.06f;
    uint8_t passes = 6;
    std::string baseConfig = "splitTree.json";
    unsigned seed = 0;
  };

  std::vector<Branch> branches;
  std::vector<TreeMeshVertex> mesh;

  // test L-System generation
  void testLSystem();

  // initiate and generate tree, for outer usage
  void generateTree(const TreeParameters params, const LSystem ls);

private:
  // lSystem deals with the generation of the L-System that the tree is based on
  LSystem lSystem;

  // treeParameters deals with the generation of the tree structure and mesh
  // from the L-System
  TreeParameters treeParameters;

  // Collision handling class
  OccupancyGrid collisionGrid;

  // keep track of max dephth - dynamic variable that should exist outside of
  // tree parameters ( depends on RNG)
  uint16_t maxDepth;

  // rng, might change source of rng later
  float RNG();

  // initiate a new tree with these parameters
  void initiateTree(const TreeParameters& params, const LSystem& ls);

  // reads a configuration file for tree generation and loads into
  // treeParameters and lSystem
  void loadTreeParameters(const std::string &fileName);

  // L-System functions
  std::string resolveLSystem(const uint8_t passes);

  uint16_t timeOffset = 0;

  // generate branch structure from lSystem
  void turtleGeneration(vec3 origin, quat originRotation);

  /*void instructTurtle(char instruction, std::vector<uint32_t> &branchStack,
                      std::vector<vec3> &positionStack,
                      std::vector<quat> &rotationStack,
                      std::vector<float> &thicknessStack, uint16_t &depth,
                      vec3 &turtlePosition, quat &turtleRotation,
                      float &turtleThickness);*/
  void advanceTurtleState(const TurtleState& state, std::vector<TurtleState>& nextLayer);

  Branch generateSingleBranch(const vec3& origin, const quat& originRotation, const uint32_t parentIndex, const uint16_t depth);
  void generateBranchMesh(const uint32_t branchIndex, const vec3 &origin,
                          const quat &orientation, const float length,
                          const float thickness);

  quat rotateBranchAbsolute(const quat &rotation, const vec3 amount);
  quat applyHeliotropism(const quat& rotation, const float amount);
  vec3 rotateVector(const vec3 &vector, const quat &rotation);
};
