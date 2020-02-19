#include "types.h"

#include <iostream>

Scene::Scene(State egoState_) : egoState(egoState_) {}

void Scene::print() const {
  for (const auto &objectState : states) {
    std::cout << "object id: " << objectState.first << std::endl;
    std::cout << "s: " << objectState.second.s << std::endl;
    std::cout << "v: " << objectState.second.v << std::endl;
    std::cout << "a: " << objectState.second.a << std::endl;
  }
  std::cout << "ego state: " << std::endl;
  std::cout << "s: " << egoState.s << std::endl;
  std::cout << "v: " << egoState.v << std::endl;
  std::cout << "a: " << egoState.a << std::endl;
}

void State::print() const {
  std::cout << "s: " << s << " v: " << v << " a: " << a << std::endl;
}
