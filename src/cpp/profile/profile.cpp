#include "grpl/profile/profile.h"

using namespace grpl::profile;

void intern::profile_internal::calculate(intern::profile_internal::segment_raw *out, intern::profile_internal::segment_raw *last, double time) {
  intern::profile_internal::segment_raw tmp = { last->time, last->dist, last->vel, last->acc };

  double dt = time - last->time;
  int slice_count = static_cast<int>(dt / _timeslice_dt);

  int i;
  if (slice_count < 1) {
    calculate_single(&tmp, &tmp, time);
  } else {
    for (i = 1; i <= slice_count; i++) {
      double time = tmp.time + _timeslice_dt;
      calculate_single(&tmp, &tmp, time);
    }
  }
  out->time = tmp.time;
  out->dist = tmp.dist;
  out->vel = tmp.vel;
  out->acc = tmp.acc;
}