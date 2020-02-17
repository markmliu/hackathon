#pragma once

#include <map>
#include <vector>

enum class VariableType {
  Z = 0, // Observation
  X = 1, // Actual state
  M = 2, // Maneuver (assert or yield)
  A = 3, // Action (trajectory)
};

using ObjectId = int;

struct NodeId {
  NodeId(ObjectId objectId_, int timestep_, VariableType variableType_);
  ObjectId objectId;
  int timestep;
  VariableType variableType;
};
bool operator<(const NodeId &lhs, const NodeId &rhs);
struct Node {
  Node(NodeId id_);
  // Redundantly stored for convenience
  NodeId id;
  std::vector<NodeId> children;
};

class DynamicBayesianNetwork {
public:
  void AddNode(NodeId nodeId);
  void AddEdge(NodeId source, NodeId target);

private:
  std::map<NodeId, Node> nodes_;
};

struct State {
  State(double s_, double v_, double a_) : s(s_), v(v_), a(a_) {}
  // s should be in absolute frame
  double s;
  double v;
  double a;
};
struct Scene {
  std::map<ObjectId, State> states;
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
