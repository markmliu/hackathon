#pragma once

#include "types.h"

#include <map>
#include <vector>

class DynamicBayesianNetwork {
public:
  void AddNode(NodeId nodeId);
  void AddEdge(NodeId source, NodeId target);

private:
  std::map<NodeId, Node> nodes_;
};

class DBNPredictor {
public:
  DBNPredictor() { current_timestep_ = 0; }
  void UpdateScene(const Scene &scene);
  void AddActor(ObjectId objectId);
  void RemoveActor(ObjectId objectid);

private:
  // Represents current timestep.
  int current_timestep_;
  DynamicBayesianNetwork dbn_;
};
