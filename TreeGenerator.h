#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
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

    uint32_t parent;
  };

  // Mesh buffer input
  struct TreeMeshVertex {
    vec3 position;
    vec3 normal;
    uint32_t parentBranch; // index of parent branch in the buffer
  };

  // parameters that determine tree generation (branch and mesh representations)
  struct TreeParameters {
    std::vector<Branch> branches;

    float trunkTwist = 0.1f;
    float trunkBend = 0.2f;

    float branchTwist = 0.2f;
    float branchBend = 0.4f;
  };

  // parameters and variables that govern L-System generation
  struct LSystem {
    uint8_t depth;
    std::string lState;
    std::map<char, Rule> ruleSet;
  };

  // rng seed : public so it is easily modified in GUI
  unsigned seed;

  // initiate a new tree with these parameters
  void initiateTree(TreeParameters params, LSystem ls, unsigned seed);

  // test L-System generation
  void testLSystem();

private:
  // treeParameters deals with the generation of the tree structure and mesh
  // from the L-System
  TreeParameters treeParameters;

  // lSystem deals with the generation of the L-System that the tree is based on
  LSystem lSystem;

  // L-System functions
  std::string resolveLSystem(int passes);

  // rng, might change source of rng later
  float RNG();

  // reads a configuration file for tree generation and loads into
  // treeParameters and lSystem
  void loadTreeParameters(std::string fileName);

  // generate branch structure from lSystem
  void turtleGeneration(vec3 origin, quat originRotation);

  void instructTurtle(char instruction, std::vector<uint32_t> &branchStack,
                      std::vector<vec3> &positionStack,
                      std::vector<quat> &rotationStack, uint16_t &depth,
                      vec3 &turtlePosition, quat &turtleRotation);

  Branch generateSingleBranch(vec3 origin, quat originRotation, uint16_t depth);

  quat rotateBranch(quat &rotation, vec3 amount);
  vec3 rotateVector(const vec3 &vector, const quat &rotation);
};
