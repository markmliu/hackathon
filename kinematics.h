#include "types.h"

// Different strategies that actor will make.
enum Strategy {
  // Constant acceleration
  CONSTANT_ACCEL = 0,
  // No accel, then max accel pretty late.
  MAX_ACCEL_LATE = 1,
};

void EvolveState(double dt, State *state);
void EvolveScene(double dt, Scene *scene,
                 Strategy egoStrategy = Strategy::CONSTANT_ACCEL,
                 Strategy objectStrategy = Strategy::CONSTANT_ACCEL);
