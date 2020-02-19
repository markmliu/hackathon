#include "types.h"

#include <iostream>

NodeId::NodeId(ObjectId objectId_, int timestep_, VariableType variableType_)
    : objectId(objectId_), timestep(timestep_), variableType(variableType_) {}

Node::Node(NodeId id_) : id(id_) {}

bool operator<(const NodeId &lhs, const NodeId &rhs) {
  return std::tie(lhs.timestep, lhs.objectId, lhs.variableType) <
         std::tie(rhs.timestep, rhs.objectId, rhs.variableType);
}

Scene::Scene(State egoState_) : egoState(egoState_) {}

void Scene::print() const {
  for (const auto &objectState : states) {
    std::cout << "object id: " << objectState.first << std::endl;
    std::cout << "s: " << objectState.second.s << std::endl;
    std::cout << "v: " << objectState.second.v << std::endl;
    std::cout << "a: " << objectState.second.a << std::endl;
  }
  std::cout << "ego state: " << std::endl;
  std::cout << "s: " << egoState.s << std::endl;
  std::cout << "v: " << egoState.v << std::endl;
  std::cout << "a: " << egoState.a << std::endl;
}
