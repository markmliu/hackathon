#include "kinematics.h"
#include "particle_filter.h"

#include <iostream>

const int NUM_PARTICLES = 500;

int main() {
  ParticleFilter pf(NUM_PARTICLES);
  Scene scene(State(/*s=*/0,
                    /*v=*/30,
                    /*a=*/0));
  scene.states.emplace(/*objectId=*/123, State(/*s=*/0,
                                               /*v=*/30,
                                               /*a=*/0));
  scene.criticalPointS = 100;
  scene.timestamp = 1.0;

  pf.Init(scene);
  // std::pair<Scene, Scene> yieldingBeatingScenes = pf.GetMeanScenes();
  // std::cout << "yielding scene: " << std::endl;
  // yieldingBeatingScenes.first.print();

  // std::cout << "beating scene: " << std::endl;
  // yieldingBeatingScenes.second.print();

  {
    // Let's evolve the scene at a dt of 0.5
    // Assume the actor is in same relative position, but we
    // are 15m closer to the critical point.
    double dt = 0.5;
    Scene updatedScene = scene;
    EvolveState(dt, &updatedScene.egoState);
    EvolveState(dt, &updatedScene.states.at(123));
    updatedScene.timestamp = 1.5;

    pf.Update(updatedScene);
  }
}
