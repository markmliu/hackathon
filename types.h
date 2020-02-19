#pragma once

#include <map>
#include <vector>

enum Maneuver {
  YIELDING = 0,
  BEATING = 1,
  IGNORING = 2,
  NUM_MANEUVERS = 3,
};

using ObjectId = int;

const ObjectId EGO_ID = 1;

// Represents a predicted state.
struct State {
  State(double s_, double v_, double a_) : s(s_), v(v_), a(a_) {}
  // s should be in absolute frame
  double s;
  double v;
  double a;
  Maneuver m;
  void print() const;
};
struct Scene {
  Scene(State egoState_);
  std::map<ObjectId, State> states;
  State egoState;
  double criticalPointS;
  double timestamp;
  void print() const;
};
