#include "mm_bringup/sim_model.hpp"
#include <iostream>

int main() {
  try { std::cout << risk_mm::prepare_simulation_model(); }
  catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
