#include "grpl_pathfinder_path_AbstractSpline2d.h"
#include "jnieigen.h"
#include "jnihandle.h"

#include <grpl/pf/path/spline.h>

using namespace grpl::pf::path;

using spline_t = spline<2>;

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_path_AbstractSpline2d_position(JNIEnv *env, jclass claz,
                                                                                   jlong handle, jdouble t) {
  spline_t::vector_t vec = jni_handle<spline_t>(env, handle)->position(t);
  return eigen_create_jdoubleArray<spline_t::vector_t>(env, vec);
}

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_path_AbstractSpline2d_derivative(JNIEnv *env, jclass claz,
                                                                                   jlong handle, jdouble t) {
  spline_t::vector_t vec = jni_handle<spline_t>(env, handle)->derivative(t);
  return eigen_create_jdoubleArray<spline_t::vector_t>(env, vec);
}

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_path_AbstractSpline2d_rotation(JNIEnv *env, jclass claz,
                                                                                   jlong handle, jdouble t) {
  spline_t::vector_t vec = jni_handle<spline_t>(env, handle)->rotation(t);
  return eigen_create_jdoubleArray<spline_t::vector_t>(env, vec);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_path_AbstractSpline2d_curvature(JNIEnv *env, jclass claz,
                                                                               jlong handle, jdouble t) {
  return jni_handle<spline_t>(env, handle)->curvature(t);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_path_AbstractSpline2d_free(JNIEnv *env, jclass claz,
                                                                       jlong handle) {
  delete jni_handle<spline_t>(env, handle);
}