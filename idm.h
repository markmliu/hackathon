// use this for asserting/yielding to actor

// desired velocity {\displaystyle v_{0}}v_{0}: the velocity the vehicle would
// drive at in free traffic minimum spacing {\displaystyle s_{0}}s_{0}: a
// minimum desired net distance. A car can't move if the distance from the car
// in the front is not at least {\displaystyle s_{0}}s_{0} desired time headway
// {\displaystyle T}T: the minimum possible time to the vehicle in front
// acceleration {\displaystyle a}a: the maximum vehicle acceleration
// comfortable braking deceleration {\displaystyle b}b: a positive number

// particles can be on desired time headway

// Should only be applied to a vehicle in same lane in front of target object
// Won't use this for v1 since we will only assume one merging vehicle.
double getIDMAccel(double initialVelocity, double distanceToPrecedingAgent,
                   double precedingAgentVelocity, double desiredVelocity,
                   double desiredTimeHeadway, double comfortableBrakingDecel,
                   double maxAccel, double minimumSpacing);

double getIDMAccelFreeRoad(double initialVelocity, double desiredVelocity,
                           double maxAccel);

// These two methods should be extended to multiple actors
// Use some placeholder stuff here which can be replaced with more accurate
// heuristics.
// Assumptions:
// Actor perfectly knows ego velocity
// Actor assumes ego will maintain constant velocity.
// By critical point, actor wants to:
// - be at ego vehicle vel
// - ego vehicle in front/behind plus some safe buffer
// -
// aggressivenessTimePad can be used to tweak their expectation
// If we want to model a driver who thinks we will yield to them,
// set a higher aggressivenessTimePad
// TODO: account for vehicle length

double getMinAccelInFrontOfEgoAtConflictRegion(
    double actorDistanceToConflictPoint, double actorVelocity,
    double egoDistanceToConflictPoint, double egoVelocity,
    double aggressivenessTimePad = 0.0);

double getMaxAccelBehindEgoAtConflictRegion(double actorDistanceToConflictPoint,
                                            double actorVelocity,
                                            double egoDistanceToConflictPoint,
                                            double egoVelocity);
