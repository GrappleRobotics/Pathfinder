#include "grpl_pathfinder_path_AbstractCurve2d.h"
#include "jnieigen.h"
#include "jnihandle.h"

#include <grpl/pf/path/curve.h>

using namespace grpl::pf::path;

using curve_t = curve<2>;

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_position(JNIEnv *env, jclass claz,
                                                                                  jlong handle, jdouble s) {
  curve_t::vector_t vec = jni_handle<curve_t>(env, handle)->position(s);
  return eigen_create_jdoubleArray<curve_t::vector_t>(env, vec);
}

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_derivative(JNIEnv *env, jclass claz,
                                                                                  jlong handle, jdouble s) {
  curve_t::vector_t vec = jni_handle<curve_t>(env, handle)->derivative(s);
  return eigen_create_jdoubleArray<curve_t::vector_t>(env, vec);
}

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_rotation(JNIEnv *env, jclass claz,
                                                                                  jlong handle, jdouble s) {
  curve_t::vector_t vec = jni_handle<curve_t>(env, handle)->rotation(s);
  return eigen_create_jdoubleArray<curve_t::vector_t>(env, vec);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_curvature(JNIEnv *env, jclass claz,
                                                                              jlong handle, jdouble s) {
  return jni_handle<curve_t>(env, handle)->curvature(s);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_dcurvature(JNIEnv *env, jclass claz,
                                                                                     jlong   handle,
                                                                                     jdouble s) {
  return jni_handle<curve_t>(env, handle)->dcurvature(s);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_length(JNIEnv *env, jclass claz,
                                                                           jlong handle) {
  return jni_handle<curve_t>(env, handle)->length();
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_path_AbstractCurve2d_free(JNIEnv *env, jclass claz,
                                                                      jlong handle) {
  delete jni_handle<curve_t>(env, handle);
}