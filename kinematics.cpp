#include "kinematics.h"

void EvolveState(double dt, State *state) {
  state->s += (state->v * dt) + (.5 * state->a * dt * dt);
  state->v += state->a * dt;
}
