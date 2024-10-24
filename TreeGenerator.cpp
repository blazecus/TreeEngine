#include "TreeGenerator.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/random.hpp>

#include <iostream>

using Branch = TreeGenerator::Branch;
using TreeParameters = TreeGenerator::TreeParameters;
using LSystem = TreeGenerator::LSystem;
using Rule = TreeGenerator::Rule;

void TreeGenerator::initiateTree(TreeParameters params, LSystem ls,
                                 unsigned newSeed) {

  seed = newSeed;
  treeParameters = params;
  lSystem = ls;
  // initiate rng seed
  srand(seed);

  return;
}

// resolveLSystem : runs a given amount of passes of the L-System stored in
// the TreeGenerator class, returns the string
std::string TreeGenerator::resolveLSystem(int passes) {
  std::string currentPass = lSystem.lState;
  std::string nextPass = "";
  for (int pass = 0; pass < passes; pass++) {
    uint32_t branchDepth = 0;
    nextPass = "";
    // loop through each character and apply rule
    for (char token : currentPass) {
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
          if (RNG() < cumulativeChance) {
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

  return currentPass;
}

// RNG method so I can change rng process later
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
  treeParameters.branches.push_back(
      generateSingleBranch(origin, originRotation, 0));
  uint16_t depth = 0;
  // these stacks keep track of where we
  std::vector<uint32_t> branchStack{0};
  std::vector<vec3> positionStack{origin};
  std::vector<quat> rotationStack{originRotation};

  // need to keep track of turtle state as it can move without saving to the
  // stack
  vec3 turtlePosition;
  quat turtleRotation;

  for (char instruction : lSystem.lState) {
    instructTurtle(instruction, branchStack, positionStack, rotationStack,
                   depth, turtlePosition, turtleRotation);
  }
}

void TreeGenerator::instructTurtle(char instruction,
                                   std::vector<uint32_t> &branchStack,
                                   std::vector<vec3> &positionStack,
                                   std::vector<quat> &rotationStack,
                                   uint16_t &depth, vec3 &turtlePosition,
                                   quat &turtleRotation) {

  // retrieve the branch we're currently traversing (top level branch in branch
  // stack)

  if (instruction == 'F') {
    // move turtle forward with some rotation
  } else if (instruction == '[') {
    // open bracket means a new branch is placed, and recursively drawn
    depth++;

    // place origin of next branch at local position, not global position
    Branch addBranch = generateSingleBranch(
        turtlePosition - positionStack[positionStack.size() - 1],
        turtleRotation - rotationStack[rotationStack.size() - 1], depth);
    addBranch.parent = branchStack[branchStack.size() - 1];

    // update stacks to new branch location
    branchStack.push_back(treeParameters.branches.size());
    positionStack.push_back(turtlePosition);
    rotationStack.push_back(turtleRotation);

    // add branch to branches
    treeParameters.branches.push_back(addBranch);

    turtleRotation =
        addBranch.orientation + rotationStack[rotationStack.size() - 2];

  } else if (instruction == ']') {
    // closed bracket means branch is ended, and turtle returns to the beginning
    // of the branch
    depth--;

    // get rid of stored branch
    branchStack.pop_back();
    positionStack.pop_back();
    rotationStack.pop_back();

    turtlePosition = positionStack[positionStack.size() - 1];
    turtleRotation = rotationStack[rotationStack.size() - 1];
  }
}

Branch TreeGenerator::generateSingleBranch(vec3 origin, quat originRotation,
                                           uint16_t depth) {
  Branch newBranch;
  newBranch.origin = origin;
  newBranch.orientation = originRotation; // TODO: apply some randomization here

  // depth should be the main factor affecting randomness

  // apply random rotation according to depth
  return newBranch;
}
