#include "grpl_pathfinder_profile_AbstractNativeProfile.h"
#include "jnihandle.h"
#include "profile/jniprofile.h"

#include <grpl/pf/profile/profile.h>

using namespace grpl::pf::profile;

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_profile_AbstractNativeProfile_calculateNative(
    JNIEnv *env, jclass claz, jlong handle, jdoubleArray last, jdouble lastTime, jdouble time) {
  state st = jni_array_to_native_state(env, lastTime, last);

  st = jni_handle<profile>(env, handle)->calculate(st, time);
  return jni_state_to_array(env, st);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractNativeProfile_setGoal(JNIEnv *env, jclass claz,
                                                                            jlong handle, jdouble goal) {
  jni_handle<profile>(env, handle)->set_goal(goal);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_profile_AbstractNativeProfile_getGoal(JNIEnv *env, jclass claz,
                                                                               jlong handle) {
  return static_cast<jdouble>(jni_handle<profile>(env, handle)->get_goal());
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractNativeProfile_setTimeslice(JNIEnv *env, jclass claz,
                                                                                 jlong handle, jdouble ts) {
  jni_handle<profile>(env, handle)->set_timeslice(ts);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_profile_AbstractNativeProfile_getTimeslice(JNIEnv *env, jclass claz,
                                                                                    jlong handle) {
  return static_cast<jdouble>(jni_handle<profile>(env, handle)->get_timeslice());
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractNativeProfile_applyLimit(JNIEnv *env, jclass claz,
                                                                               jlong handle, jint deriv,
                                                                               jdouble min, jdouble max) {
  jni_handle<profile>(env, handle)->apply_limit(deriv, min, max);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractNativeProfile_free(JNIEnv *env, jclass claz,
                                                                         jlong handle) {
  delete jni_handle<profile>(env, handle);
}