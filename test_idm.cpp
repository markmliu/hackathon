#include "idm.h"

#include <assert.h>
#include <iostream>

int main() {
  // Test getIDMAccelFreeRoad().
  {
    double desiredVelocity = 40;
    double maxAccel = 5;

    double accel10To40 =
        getIDMAccelFreeRoad(/*initialVelocity=*/10, desiredVelocity, maxAccel);
    double accel25To40 =
        getIDMAccelFreeRoad(/*initialVelocity=*/25, desiredVelocity, maxAccel);
    std::cout << accel10To40 << std::endl;
    std::cout << accel25To40 << std::endl;
    assert(accel10To40 > accel25To40);
  }

  // Test getIDMAccel()

  // Things to test -
  // Initial position near/in front of ego slows to behind us
  {
    // Same speed
    double desiredVelocity = 70;
    double maxAccel = 5;

    // ~40mph
    double initialVelocity = 20;
    // TODO: try making this negative
    double distanceToPrecedingAgent = 20;

    // ~60mph
    double precedingAgentVelocity = 30;
    double desiredTimeHeadway = 0.5;
    double comfortableBrakingDecel = 4.0;
    // not sure what this should be
    double minimumSpacing = 0;
    double accel =
        getIDMAccel(initialVelocity, distanceToPrecedingAgent,
                    precedingAgentVelocity, desiredVelocity, desiredTimeHeadway,
                    comfortableBrakingDecel, maxAccel, minimumSpacing);
    std::cout << accel << std::endl;
  }
  return 0;
}
