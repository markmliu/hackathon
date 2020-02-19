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

const ObjectId EGO_ID = 1;

// hack to get around boost::optional issue
const ObjectId NO_OBJECT = -1;

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

// Represents a predicted state.
struct State {
  State(double s_, double v_, double a_) : s(s_), v(v_), a(a_) {}
  // s should be in absolute frame
  double s;
  double v;
  double a;
  // Represents the object that is being "followed".
  // Preprocessing should only allow:
  // - The vehicle currently in front of object
  // - Any vehicle in merging lane
  ObjectId leadObject;
  ObjectId rearObject;
};
struct Scene {
  std::map<ObjectId, State> states;
  // relative to ego vehicle.
  double distToCriticalPoint;
  double egoVelocity;
  void print() const;
};
