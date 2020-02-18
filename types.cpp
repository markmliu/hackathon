#include "types.h"

NodeId::NodeId(ObjectId objectId_, int timestep_, VariableType variableType_)
    : objectId(objectId_), timestep(timestep_), variableType(variableType_) {}

Node::Node(NodeId id_) : id(id_) {}

bool operator<(const NodeId &lhs, const NodeId &rhs) {
  return std::tie(lhs.timestep, lhs.objectId, lhs.variableType) <
         std::tie(rhs.timestep, rhs.objectId, rhs.variableType);
}
