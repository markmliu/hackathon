#include "particle_filter.h"

int main() {
  ParticleFilter pf;
  Scene scene;
  scene.states.emplace(/*objectId=*/123, State(/*s=*/-5,
                                               /*v=*/10,
                                               /*a=*/0));
  pf.Init(scene);
  pf.PrintParticles();
}
