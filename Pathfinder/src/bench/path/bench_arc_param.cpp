#include "grpl/pf/path/arc_parameterizer.h"
#include "grpl/pf/path/hermite.h"

#include <vector>

using namespace grpl::pf;
using namespace grpl::pf::path;

#include <benchmark/benchmark.h>

static void BM_ArcParamHermite(benchmark::State &state) {
  using hermite_t = hermite_quintic;

  hermite_t::waypoint start{{2, 2}, {5, 0}, {0, 0}}, end{{5, 5}, {5, 5}, {0, 0}};
  hermite_t           hermite(start, end);

  int num_curves = 0, num_iter = 0;

  for (auto _ : state) {
    state.PauseTiming();
    std::vector<arc_parameterizer::curve_t> curves;
    arc_parameterizer                       param;
    double                                  sensitivity = 1.0 / static_cast<double>(state.range(0));
    param.configure(sensitivity, sensitivity);
    curves.reserve(param.curve_count(hermite));  // Ensure the timing isn't affected by reallocs
    state.ResumeTiming();

    param.parameterize(hermite, std::back_inserter(curves), curves.max_size());
    num_curves += curves.size();
    num_iter++;
  }

  state.counters["NumCurves"] = num_curves / num_iter;
  state.SetComplexityN(state.range(0));
}

BENCHMARK(BM_ArcParamHermite)->Arg(1)->Arg(10)->Arg(100)->Arg(1000)->Complexity()->Unit(benchmark::kMillisecond);