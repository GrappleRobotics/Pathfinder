#include "grpl_pathfinder_path_HermiteCubic.h"
#include "grpl_pathfinder_path_HermiteQuintic.h"
#include "jnieigen.h"
#include "jnihandle.h"

#include <grpl/pf/path/hermite.h>

using namespace grpl::pf::path;

static typename hermite_cubic::waypoint cubic_waypoint(JNIEnv *env, jdoubleArray wp) {
  double *wparr = env->GetDoubleArrayElements(wp, nullptr);

  typename hermite_quintic::vector_t pos{wparr[0], wparr[1]}, tang{wparr[2], wparr[3]};
  env->ReleaseDoubleArrayElements(wp, wparr, 0);
  return typename hermite_cubic::waypoint{pos, tang};
}

static typename hermite_quintic::waypoint quintic_waypoint(JNIEnv *env, jdoubleArray wp) {
  double *wparr = env->GetDoubleArrayElements(wp, nullptr);

  typename hermite_quintic::vector_t pos{wparr[0], wparr[1]}, tang{wparr[2], wparr[3]},
      dtang{wparr[4], wparr[5]};
  env->ReleaseDoubleArrayElements(wp, wparr, 0);
  return typename hermite_quintic::waypoint{pos, tang, dtang};
}

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_path_HermiteCubic_allocate(JNIEnv *env, jclass clz,
                                                                        jdoubleArray start,
                                                                        jdoubleArray end) {
  using hermite_t = hermite_cubic;

  hermite_t::waypoint wpstart = cubic_waypoint(env, start);
  hermite_t::waypoint wpend   = cubic_waypoint(env, end);

  return jni_as_handle<hermite_t>(new hermite_t(wpstart, wpend));
}

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_path_HermiteQuintic_allocate(JNIEnv *env, jclass clz,
                                                                          jdoubleArray start,
                                                                          jdoubleArray end) {
  using hermite_t = hermite_quintic;

  hermite_t::waypoint wpstart = quintic_waypoint(env, start);
  hermite_t::waypoint wpend   = quintic_waypoint(env, end);

  return jni_as_handle<hermite_t>(new hermite_t(wpstart, wpend));
}