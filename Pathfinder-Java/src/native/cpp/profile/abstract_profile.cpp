#include "grpl_pathfinder_profile_AbstractProfile.h"
#include "jnihandle.h"

#include <grpl/profile/profile.h>

using namespace grpl::profile;

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractProfile_setGoal(JNIEnv *env, jobject obj,
                                                                            jdouble goal) {
  jni_get_handle<profile_base>(env, obj)->set_goal(goal);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_profile_AbstractProfile_getGoal(JNIEnv *env, jobject obj) {
  return static_cast<jdouble>(jni_get_handle<profile_base>(env, obj)->get_goal());
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractProfile_setTimeslice(JNIEnv *env, jobject obj,
                                                                                 jdouble ts) {
  jni_get_handle<profile_base>(env, obj)->set_timeslice(ts);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_profile_AbstractProfile_getTimeslice(JNIEnv *env,
                                                                                    jobject obj) {
  return static_cast<jdouble>(jni_get_handle<profile_base>(env, obj)->get_timeslice());
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractProfile_applyLimit(JNIEnv *env, jobject obj,
                                                                               jint deriv, jdouble min,
                                                                               jdouble max) {
  jni_get_handle<profile_base>(env, obj)->apply_limit(deriv, min, max);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_profile_AbstractProfile_destroy(JNIEnv *env, jobject obj) {
  profile_base *profile = jni_get_handle<profile_base>(env, obj);
  delete profile;
  profile = static_cast<profile_base *>(nullptr);
  jni_set_handle(env, obj, profile);
}