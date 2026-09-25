#include <cstdlib>
#include <iostream>
#include <limits>
#include "small_gicp_relocalization/registration_quality.hpp"

int main()
{
  using small_gicp_relocalization::registrationQualityAcceptable;
  using small_gicp_relocalization::initialCorrectionAcceptable;
  auto check = [](bool condition, const char * name) {
    if (!condition) { std::cerr << "FAIL " << name << '\n'; std::exit(1); }
  };
  check(registrationQualityAcceptable(false, 800, 1000, 8.0, .05, .3, .3),
    "iteration limit with good error must be accepted");
  check(!registrationQualityAcceptable(false, 800, 1000, 80.0, .05, .3, .3),
    "iteration limit with poor error must be rejected");
  check(!registrationQualityAcceptable(true, 100, 1000, 1.0, .05, .3, .3),
    "convergence alone cannot override low overlap");
  check(!registrationQualityAcceptable(true, 800, 1000, 400.0, .05, .3, .3),
    "convergence alone cannot override excessive fitness error");
  check(!registrationQualityAcceptable(true, 800, 1000,
    std::numeric_limits<double>::quiet_NaN(), .05, .3, .3), "NaN must fail");
  check(!registrationQualityAcceptable(true, 0, 0, 0.0, .05, .3, .3), "empty input must fail");
  check(!initialCorrectionAcceptable(-.122, -3.702, -.120, 1.0, .35),
    "logged 3.704m startup jump must be rejected despite good fitness");
  check(initialCorrectionAcceptable(.4, -.3, .08, 1.0, .35),
    "nearby correct registration remains usable");
  check(!initialCorrectionAcceptable(.1, -.1, 3.141592653589793, 1.0, .35),
    "opposite-heading false match must be rejected");
  check(initialCorrectionAcceptable(.1, -.1, 6.283185307179586-.01, 1.0, .35),
    "angle wraparound is handled");
  std::cout << "PASS startup prior: logged jump, nearby match, yaw reversal, angle wrap\n";
  std::cout << "PASS registration quality: iteration-limit, overlap, fitness, NaN, empty input\n";
}
