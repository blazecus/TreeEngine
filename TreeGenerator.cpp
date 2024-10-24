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


// note for later - branches need local coordinates. relative positions can be calculated as forces are, then vertex positions have to be calculated backwards
void TreeGenerator::turtleGeneration(){
  std::vector<uint32_t> branchStack;
  std::vector<vec3> positionStac
  for(char instruction : lSystem.lState){
    instructTurtle
  }
}

void TreeGenerator::instructTurtle(char instruction, std::vector<uint32_t> &branchStack){
  if(instruction == 'F'){
    // move turtle forward
  }
  else if(instruction == '['){

  }
  else if (instruction == ']'){

  }
  else if
}
