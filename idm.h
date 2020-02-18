// use this for asserting/yielding to actor

// desired velocity {\displaystyle v_{0}}v_{0}: the velocity the vehicle would
// drive at in free traffic minimum spacing {\displaystyle s_{0}}s_{0}: a
// minimum desired net distance. A car can't move if the distance from the car
// in the front is not at least {\displaystyle s_{0}}s_{0} desired time headway
// {\displaystyle T}T: the minimum possible time to the vehicle in front
// acceleration {\displaystyle a}a: the maximum vehicle acceleration
// comfortable braking deceleration {\displaystyle b}b: a positive number

// particles can be on desired time headway

double getIDMAccel(double initialVelocity, double distanceToPrecedingAgent,
                   double precedingAgentVelocity, double desiredVelocity,
                   double desiredTimeHeadway, double comfortableBrakingDecel,
                   double maxAccel, double minimumSpacing);

double getIDMAccelFreeRoad(double initialVelocity, double desiredVelocity,
                           double maxAccel);
