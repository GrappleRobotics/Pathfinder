#include "grpl_pathfinder_path_HermiteCubic.h"
#include "grpl_pathfinder_path_HermiteQuintic.h"
#include "jnieigen.h"
#include "jnihandle.h"

#include <grpl/pf/path/hermite.h>

using namespace grpl::pf::path;

template <typename T>
static typename T::waypoint hermite_create_waypoint(JNIEnv *env, jdoubleArray wp) {
  double *             wparr = env->GetDoubleArrayElements(wp, nullptr);
  typename T::vector_t pos{wparr[0], wparr[1]}, tang{wparr[2], wparr[3]}, tang_slope{wparr[4], wparr[5]};
  env->ReleaseDoubleArrayElements(wp, wparr, 0);
  return typename T::waypoint{pos, tang, tang_slope};
}

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_path_HermiteCubic_allocate(JNIEnv *env, jclass clz,
                                                                        jdoubleArray start,
                                                                        jdoubleArray end) {
  using hermite_t = hermite_cubic;

  hermite_t::waypoint wpstart = hermite_create_waypoint<hermite_t>(env, start);
  hermite_t::waypoint wpend   = hermite_create_waypoint<hermite_t>(env, end);

  return jni_as_handle<hermite_t>(new hermite_t(wpstart, wpend));
}

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_path_HermiteQuintic_allocate(JNIEnv *env, jclass clz,
                                                                          jdoubleArray start,
                                                                          jdoubleArray end) {
  using hermite_t = hermite_quintic;

  hermite_t::waypoint wpstart = hermite_create_waypoint<hermite_t>(env, start);
  hermite_t::waypoint wpend   = hermite_create_waypoint<hermite_t>(env, end);

  return jni_as_handle<hermite_t>(new hermite_t(wpstart, wpend));
}