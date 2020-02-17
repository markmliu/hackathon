#include "dynamic_bayesian_network.h"

#include <iostream>

NodeId::NodeId(ObjectId objectId_, int timestep_, VariableType variableType_)
    : objectId(objectId_), timestep(timestep_), variableType(variableType_) {}

Node::Node(NodeId id_) : id(id_) {}

bool operator<(const NodeId &lhs, const NodeId &rhs) {
  return std::tie(lhs.timestep, lhs.objectId, lhs.variableType) <
         std::tie(rhs.timestep, rhs.objectId, rhs.variableType);
}

void DynamicBayesianNetwork::AddNode(NodeId nodeId) {
  auto ret = nodes_.emplace(nodeId, Node(nodeId));
  if (!ret.second) {
    std::cout << "Tried to add nodeid that's already been added" << std::endl;
  }
}

void DynamicBayesianNetwork::AddEdge(NodeId source, NodeId target) {
  auto it = nodes_.find(source);
  if (it == nodes_.end()) {
    std::cout << "Tried to add edge from nodeId that doesn't exist"
              << std::endl;
  }
  nodes_.at(source).children.push_back(target);
}

// DBNPredictor
void DBNPredictor::AddActor(ObjectId objectId) {
  dbn_.AddNode(NodeId(objectId, current_timestep_, VariableType::Z));
  dbn_.AddNode(NodeId(objectId, current_timestep_, VariableType::X));
  dbn_.AddNode(NodeId(objectId, current_timestep_, VariableType::M));
  dbn_.AddNode(NodeId(objectId, current_timestep_, VariableType::A));

  // Connect dependencies to previous timestep if they exist
  int previous_timestep = current_timestep_ - 1;
  dbn_.AddEdge(NodeId(objectId, previous_timestep, VariableType::X),
               NodeId(objectId, current_timestep_, VariableType::X));
  dbn_.AddEdge(NodeId(objectId, previous_timestep, VariableType::A),
               NodeId(objectId, current_timestep_, VariableType::X));
  dbn_.AddEdge(NodeId(objectId, previous_timestep, VariableType::M),
               NodeId(objectId, current_timestep_, VariableType::M));
}

void DBNPredictor::UpdateScene(const Scene &scene) {
  for (const auto objectState : scene.states) {
    AddActor(objectState.first);
  }
  current_timestep_++;
}
