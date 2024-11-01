#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <json.hpp>
#include <map>
#include <string>
#include <vector>

class TreeGenerator {
public:
  // (Just aliases to make notations lighter)
  using vec3 = glm::vec3;
  using vec2 = glm::vec2;
  using mat3x3 = glm::mat3x3;
  using quat = glm::quat;

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

    float branchTwist = 0.4f;
    float branchBend = 0.8f;

    float branchLength = 0.2f;
    float branchLengthDepthFactor = 2.0f;
    float baseBranchThickness = 0.01f;
    float branchThicknessDepthFactor = 0.2f;

    float heliotropismChance = 0.3f;
  };

  // parameters and variables that govern L-System generation
  struct LSystem {
    uint8_t depth;
    std::string lState;
    std::map<char, Rule> ruleSet;
    float depthBias = 0.06f;
  };

  // rng seed : public so it is easily modified in GUI
  unsigned seed;

  std::vector<Branch> branches;
  std::vector<TreeMeshVertex> mesh;

  // initiate a new tree with these parameters
  void initiateTree(TreeParameters params, LSystem ls, unsigned seed);

  // test L-System generation
  void testLSystem();

  // reads a configuration file for tree generation and loads into
  // treeParameters and lSystem
  void loadTreeParameters(const std::string &fileName);

  // L-System functions
  std::string resolveLSystem(int passes);

  // generate branch structure from lSystem
  void turtleGeneration(vec3 origin, quat originRotation);

private:
  // lSystem deals with the generation of the L-System that the tree is based on
  LSystem lSystem;

  // treeParameters deals with the generation of the tree structure and mesh
  // from the L-System
  TreeParameters treeParameters;

  // keep track of max dephth - dynamic variable that should exist outside of
  // tree parameters ( depends on RNG)
  uint16_t maxDepth;

  // rng, might change source of rng later
  float RNG();

  void instructTurtle(char instruction, std::vector<uint32_t> &branchStack,
                      std::vector<vec3> &positionStack,
                      std::vector<quat> &rotationStack, uint16_t &depth,
                      vec3 &turtlePosition, quat &turtleRotation);

  Branch generateSingleBranch(vec3 origin, quat originRotation, uint16_t depth);
  void generateBranchMesh(const uint32_t branchIndex, const vec3 &origin,
                          const quat &orientation, const float length);

  quat rotateBranch(const quat &rotation, const vec3 amount);
  quat rotateBranchAbsolute(const quat &rotation, const vec3 amount);
  vec3 rotateVector(const vec3 &vector, const quat &rotation);
};
