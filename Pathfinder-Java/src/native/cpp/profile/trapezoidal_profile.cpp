#include "grpl_pathfinder_profile_TrapezoidalProfile.h"
#include "jnihandle.h"
#include "profile/jniprofile.h"

#include <grpl/profile/trapezoidal.h>

using namespace grpl::profile;

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_profile_TrapezoidalProfile_allocate(JNIEnv *env, jclass obj) {
  return jni_as_handle<trapezoidal>(new trapezoidal());
}

JNIEXPORT jobject JNICALL Java_grpl_pathfinder_profile_TrapezoidalProfile_calculate(JNIEnv *env, jobject obj,
                                                                                    jobject last, jdouble t) {
  trapezoidal::segment_t seg = jni_segment_to_native<trapezoidal::segment_t>(env, last);

  seg = jni_get_handle<trapezoidal>(env, obj)->calculate(seg, t);
  return jni_segment_to_java(env, seg);
}