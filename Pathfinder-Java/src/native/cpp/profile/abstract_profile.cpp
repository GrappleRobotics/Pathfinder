#include "grpl_pathfinder_profile_AbstractProfile.h"
#include "jnihandle.h"

#include <grpl/pf/profile/profile.h>

using namespace grpl::pf::profile;

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractProfile_setGoal(JNIEnv *env, jclass claz,
                                                                            jlong handle, jdouble goal) {
  jni_handle<profile_base>(env, handle)->set_goal(goal);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_profile_AbstractProfile_getGoal(JNIEnv *env, jclass claz,
                                                                               jlong handle) {
  return static_cast<jdouble>(jni_handle<profile_base>(env, handle)->get_goal());
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractProfile_setTimeslice(JNIEnv *env, jclass claz,
                                                                                 jlong handle, jdouble ts) {
  jni_handle<profile_base>(env, handle)->set_timeslice(ts);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_profile_AbstractProfile_getTimeslice(JNIEnv *env, jclass claz,
                                                                                    jlong handle) {
  return static_cast<jdouble>(jni_handle<profile_base>(env, handle)->get_timeslice());
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractProfile_applyLimit(JNIEnv *env, jclass claz,
                                                                               jlong handle, jint deriv,
                                                                               jdouble min, jdouble max) {
  jni_handle<profile_base>(env, handle)->apply_limit(deriv, min, max);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractProfile_free(JNIEnv *env, jclass claz,
                                                                         jlong handle) {
  delete jni_handle<profile_base>(env, handle);
}