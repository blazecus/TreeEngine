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

  // parameters that determine L-System and tree generation
  struct TreeParameters {
    int depth;
    int seed;
    std::vector<Rule> rules;
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

  void initiateTree();

private:
  std::map<char, Rule> ruleSet;
  TreeParameters parameters;
  std::string lState;

  // L-System functions
  std::string resolveLSystem(int passes);
  float RNG();
};
