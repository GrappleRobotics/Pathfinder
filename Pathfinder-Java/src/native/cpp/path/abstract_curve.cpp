#include "grpl_pathfinder_path_AbstractCurve2d.h"
#include "jnieigen.h"
#include "jnihandle.h"

#include <grpl/path/curve.h>

using namespace grpl::path;

using curve_t = curve<2>;

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_position(JNIEnv *env, jclass claz,
                                                                                  jlong handle, jdouble t) {
  curve_t::vector_t vec = jni_handle<curve_t>(env, handle)->position(t);
  return eigen_create_jdoubleArray<curve_t::vector_t>(env, vec);
}

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_velocity(JNIEnv *env, jclass claz,
                                                                                  jlong handle, jdouble t) {
  curve_t::vector_t vec = jni_handle<curve_t>(env, handle)->velocity(t);
  return eigen_create_jdoubleArray<curve_t::vector_t>(env, vec);
}

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_rotation(JNIEnv *env, jclass claz,
                                                                                  jlong handle, jdouble t) {
  curve_t::vector_t vec = jni_handle<curve_t>(env, handle)->rotation(t);
  return eigen_create_jdoubleArray<curve_t::vector_t>(env, vec);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_curvature(JNIEnv *env, jclass claz,
                                                                              jlong handle, jdouble t) {
  return jni_handle<curve_t>(env, handle)->curvature(t);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_curvature_1prime(JNIEnv *env, jclass claz,
                                                                                     jlong   handle,
                                                                                     jdouble t) {
  return jni_handle<curve_t>(env, handle)->curvature_prime(t);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_length(JNIEnv *env, jclass claz,
                                                                           jlong handle) {
  return jni_handle<curve_t>(env, handle)->length();
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_free(JNIEnv *env, jclass claz,
                                                                      jlong handle) {
  delete jni_handle<curve_t>(env, handle);
}