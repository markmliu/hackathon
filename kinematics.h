#include "types.h"

// Different strategies that actor will make.
enum Strategy {
  // Constant acceleration
  CONSTANT_ACCEL = 0,
  // Fixed accel, then max accel pretty late.
  MAX_ACCEL_LATE = 1,

  // Fixed accel, then small neg decel late
  ACCEL_THEN_DECEL_LATE = 2,
};

void EvolveState(double dt, State *state);

// returns actual object accel
double EvolveScene(double dt, Scene *scene,
                   Strategy egoStrategy = Strategy::CONSTANT_ACCEL,
                   Strategy objectStrategy = Strategy::CONSTANT_ACCEL);
