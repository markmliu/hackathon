#include <math>

double getIDMAccel(double initialVelocity, double distanceToPrecedingAgent,
                   double precedingAgentVelocity, double desiredVelocity,
                   double desiredTimeHeadway, double comfortableBrakingDecel,
                   double maxAccel, double delta, double minimumSpacing) {
  double deltaV = initialVelocity - precedingAgentVelocity;

  double free_road_term =
      maxAccel * pow(1 - (initialVelocity / desiredVelocity), delta);
  double interaction_term =
      -maxAccel * pow((minimumSpacing + initialVelocity * desiredTimeHeadway) /
                              distanceToPrecedingAgent +
                          (initialVelocity * deltaV) /
                              (2 * sqrt(maxAccel * comfortableBrakingDecel) *
                               distanceToPrecedingAgent),
                      2);
  return free_road_term + interaction_term;
}
