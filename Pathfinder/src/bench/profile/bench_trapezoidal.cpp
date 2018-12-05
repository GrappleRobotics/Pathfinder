#include <grpl/pf/profile/trapezoidal.h>

#include <benchmark/benchmark.h>

using namespace grpl::pf::profile;

static void BM_Profile_Trapezoidal(benchmark::State &state) {
  trapezoidal pr;
  pr.apply_limit(1, -3, 3);  // Velocity Limit = -3 to 3m/s
  pr.apply_limit(2, -3, 4);  // Acceleration limit = -3 to 4m/s
  pr.set_goal(5);            // Goal = 5m
  pr.set_timeslice(0);       // No Timeslice

  for (auto _ : state) {
    ::grpl::pf::profile::state st;
    double                 dt = 1.0 / static_cast<double>(state.range(0));
    for (double t = 0; t < 10; t += dt) {
      benchmark::DoNotOptimize(st = pr.calculate(st, t));
    }
  }

  state.SetComplexityN(state.range(0));
}

BENCHMARK(BM_Profile_Trapezoidal)->Arg(10)->Arg(100)->Arg(1000)->Arg(10000)->Complexity()->Unit(benchmark::kMillisecond);