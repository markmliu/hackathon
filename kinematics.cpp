#include "kinematics.h"

void EvolveState(double dt, State *state) {
  state->s += (state->v * dt) + (.5 * state->a * dt * dt);
  state->v += state->a * dt;
}

void EvolveScene(double dt, Scene *scene) {
  EvolveState(dt, &scene->egoState);
  EvolveState(dt, &scene->states.begin()->second);
  scene->timestamp += dt;
}
