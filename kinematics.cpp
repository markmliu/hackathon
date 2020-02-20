#include "kinematics.h"

#include <iostream>

namespace {
void ApplyStrategy(Strategy strategy, State *state, double currentTimestamp) {
  if (strategy == Strategy::CONSTANT_ACCEL) {
    // do nothing
    return;
  } else if (strategy == Strategy::MAX_ACCEL_LATE) {
    // constant time for now
    const double LATE_TIME = 5.0;
    const double MAX_ACCEL = 3.0;
    const double MAX_SPEED = 35;

    if (currentTimestamp < LATE_TIME) {
      return;
    }

    double accelToApply = std::min(MAX_ACCEL, MAX_SPEED - state->v);
    state->a = accelToApply;
    std::cout << "acceling up to " << state->a << std::endl;
    return;
  }
}
} // anonymous namespace
void EvolveState(double dt, State *state) {
  state->s += (state->v * dt) + (.5 * state->a * dt * dt);
  state->v += state->a * dt;
}

void EvolveScene(double dt, Scene *scene, Strategy egoStrategy,
                 Strategy objectStrategy) {
  ApplyStrategy(egoStrategy, &scene->egoState, scene->timestamp);
  ApplyStrategy(objectStrategy, &scene->states.begin()->second,
                scene->timestamp);
  EvolveState(dt, &scene->egoState);
  EvolveState(dt, &scene->states.begin()->second);
  scene->timestamp += dt;
}
