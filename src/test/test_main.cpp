#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <iostream>

int main(int argc, char **argv) {
#ifdef EIGEN_RUNTIME_NO_MALLOC
  Eigen::internal::set_is_malloc_allowed(false);
  std::cout << "EIGEN RUNTIME MALLOC ASSERTS ENABLED!" << std::endl;
#else
  std::cout << "WARN!!! EIGEN RUNTIME MALLOCS ARE NOT ASSERTED!" << std::endl;
#endif
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}