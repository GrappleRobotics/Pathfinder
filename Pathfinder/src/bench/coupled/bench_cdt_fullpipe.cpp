#include "grpl/pf/coupled/causal_trajectory_generator.h"
#include "grpl/pf/path/hermite.h"
#include "grpl/pf/path/arc_parameterizer.h"
#include "grpl/pf/profile/trapezoidal.h"

#include <vector>

using namespace grpl::pf;

#include <benchmark/benchmark.h>

static void BM_CDT_Full(benchmark::State &state) {
  using hermite_t = path::hermite_quintic;
  using profile_t = profile::trapezoidal;

  hermite_t::waypoint start{{2, 2}, {5, 0}, {0, 0}}, end{{5, 5}, {5, 5}, {0, 0}};
  hermite_t           hermite(start, end);

  double G = 12.75;
  transmission::dc_motor dualCIM{12.0, 5330 * 2.0 * constants::PI / 60.0 / G, 2 * 2.7, 2 * 131.0,
                                 2 * 2.41 * G};
  coupled::chassis    chassis{dualCIM, dualCIM, 0.0762, 0.5, 25.0};

  int num_iter = 0;
  int num_gens = 0;
  int num_curves = 0;
  double realtime = 0;

  double loop_time = 1.0 / static_cast<double>(state.range(0));

  for (auto _ : state) {
    state.PauseTiming();
    // Setup
    // Param
    path::arc_parameterizer param;
    param.configure(0.01, 0.01);
    // Curves Buffer
    std::vector<path::arc_parameterizer::curve_t> curves;
    curves.reserve(1024);
    // Profile
    profile_t profile;
    // Coupled
    coupled::causal_trajectory_generator gen;
    coupled::state c_state;
    state.ResumeTiming();

    // Benchmark Start
    param.parameterize(hermite, std::back_inserter(curves), curves.max_size());
    num_curves += curves.size();

    double t;
    for (t = 0; !c_state.finished && t < 5.0; t += loop_time) {
      c_state = gen.generate(chassis, curves.begin(), curves.end(), profile, c_state, t);
      std::pair<coupled::wheel_state, coupled::wheel_state> split = chassis.split(c_state);
      benchmark::DoNotOptimize(split);
      num_gens++;
    }
    realtime += t;
    num_iter++;
  }

  state.counters["NumStates"] = num_gens / num_iter;
  state.counters["NumCurves"] = num_curves / num_iter;
  state.counters["LoopDt"] = loop_time;
  state.counters["MPExecTime"] = realtime / num_iter;
  state.SetComplexityN(state.range(0));
}

BENCHMARK(BM_CDT_Full)->Arg(10)->Arg(100)->Arg(1000)->Complexity()->Unit(benchmark::kMillisecond);