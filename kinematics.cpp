#include "kinematics.h"

#include <iostream>

namespace {
double ApplyStrategy(Strategy strategy, State *state, double currentTimestamp) {
  if (strategy == Strategy::CONSTANT_ACCEL) {
    // do nothing
    return state->a;
  } else if (strategy == Strategy::MAX_ACCEL_LATE) {
    // constant time for now
    const double LATE_TIME = 5.0;
    const double MAX_ACCEL = 3.0;
    const double MAX_SPEED = 35;

    if (currentTimestamp < LATE_TIME) {
      return state->a;
    }

    double accelToApply = std::min(MAX_ACCEL, MAX_SPEED - state->v);
    state->a = accelToApply;
    std::cout << "acceling up to " << state->a << std::endl;
    return state->a;
  } else if (strategy == Strategy::ACCEL_THEN_DECEL_LATE) {
    double accelToApply;
    if (currentTimestamp < 3.0) {
      accelToApply = 2.0;
    } else if (currentTimestamp < 4.5) {
      accelToApply = 1.0;
    } else if (currentTimestamp < 5.5) {
      accelToApply = 0.0;
    } else if (currentTimestamp < 6.5) {
      accelToApply = -1.0;
    } else {
      accelToApply = -2.0;
    }
    state->a = accelToApply;
    return state->a;
  }
}
} // anonymous namespace
void EvolveState(double dt, State *state) {
  state->s += (state->v * dt) + (.5 * state->a * dt * dt);
  state->v += state->a * dt;
}

double EvolveScene(double dt, Scene *scene, Strategy egoStrategy,
                   Strategy objectStrategy) {
  ApplyStrategy(egoStrategy, &scene->egoState, scene->timestamp);
  double objectAccel = ApplyStrategy(
      objectStrategy, &scene->states.begin()->second, scene->timestamp);
  EvolveState(dt, &scene->egoState);
  EvolveState(dt, &scene->states.begin()->second);
  scene->timestamp += dt;
  return objectAccel;
}
