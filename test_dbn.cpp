#include "dynamic_bayesian_network.h"

int main() {
  DBNPredictor predictor;

  Scene initialScene;
  initialScene.states.emplace(/*objectId=*/1, State(-15.0, 3.0, 0.0));

  predictor.UpdateScene(initialScene);
  return 0;
}
