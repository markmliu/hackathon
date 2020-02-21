#include <math.h>

// Make this smaller because we expect more aggressive speedup
const int DELTA = 3;

double getIDMAccel(double initialVelocity, double distanceToPrecedingAgent,
                   double precedingAgentVelocity, double desiredVelocity,
                   double desiredTimeHeadway, double comfortableBrakingDecel,
                   double maxAccel, double minimumSpacing) {
  double deltaV = initialVelocity - precedingAgentVelocity;

  double free_road_term =
      maxAccel * pow(1 - (initialVelocity / desiredVelocity), DELTA);
  double interaction_term =
      -maxAccel * pow((minimumSpacing + initialVelocity * desiredTimeHeadway) /
                              distanceToPrecedingAgent +
                          (initialVelocity * deltaV) /
                              (2 * sqrt(maxAccel * comfortableBrakingDecel) *
                               distanceToPrecedingAgent),
                      2);
  return free_road_term + interaction_term;
}

double getIDMAccelFreeRoad(double initialVelocity, double desiredVelocity,
                           double maxAccel) {
  return maxAccel * pow(1 - (initialVelocity / desiredVelocity), DELTA);
}

double getMinAccelInFrontOfEgoAtConflictRegion(
    double actorDistanceToConflictPoint, double actorVelocity,
    double egoDistanceToConflictPoint, double egoVelocity,
    double aggressivenessTimePad) {
  double egoTimeToConflict = egoDistanceToConflictPoint / egoVelocity;
  // Pad by some amount to beat ego.
  double actorTimeToConflict = egoTimeToConflict - 0.5 + aggressivenessTimePad;
  return 2 * (actorDistanceToConflictPoint -
              (actorVelocity * actorTimeToConflict)) /
         (actorTimeToConflict * actorTimeToConflict);
}

double getMaxAccelBehindEgoAtConflictRegion(double actorDistanceToConflictPoint,
                                            double actorVelocity,
                                            double egoDistanceToConflictPoint,
                                            double egoVelocity) {
  double egoTimeToConflict = egoDistanceToConflictPoint / egoVelocity;
  // Pad by some amount to follow ego.
  double actorTimeToConflict = egoTimeToConflict + 0.5;
  return 2 * (actorDistanceToConflictPoint -
              (actorVelocity * actorTimeToConflict)) /
         (actorTimeToConflict * actorTimeToConflict);
}
