#include "grpl_pathfinder_profile_TrapezoidalProfile.h"
#include "jnihandle.h"
#include "profile/jniprofile.h"

#include <grpl/pf/profile/trapezoidal.h>

using namespace grpl::pf::profile;

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_profile_TrapezoidalProfile_allocate(JNIEnv *env, jclass obj) {
  return jni_as_handle<trapezoidal>(new trapezoidal());
}

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_profile_TrapezoidalProfile_calculateNative(
    JNIEnv *env, jobject obj, jlong handle, jdoubleArray last, jdouble lastTime, jdouble time) {
  trapezoidal::segment_t seg = jni_array_to_native_segment<typename trapezoidal::segment_t>(env, lastTime, last);

  seg = jni_handle<trapezoidal>(env, handle)->calculate(seg, time);
  return jni_segment_to_array(env, seg);
}