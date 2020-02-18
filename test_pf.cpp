#include "particle_filter.h"

#include <iostream>

const int NUM_PARTICLES = 500;

int main() {
  ParticleFilter pf(NUM_PARTICLES);
  Scene scene;
  scene.states.emplace(/*objectId=*/123, State(/*s=*/-5,
                                               /*v=*/10,
                                               /*a=*/0));
  pf.Init(scene);
  std::pair<Scene, Scene> yieldingBeatingScenes = pf.GetMeanScenes();
  std::cout << "yielding scene: " << std::endl;
  yieldingBeatingScenes.first.print();

  std::cout << "beating scene: " << std::endl;
  yieldingBeatingScenes.second.print();

  // pf.PrintParticles();
}
