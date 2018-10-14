#include "grpl_pathfinder_profile_TrapezoidalProfile.h"
#include "jnihandle.h"
#include "profile/jniprofile.h"

#include <grpl/pf/profile/trapezoidal.h>

using namespace grpl::pf::profile;

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_profile_TrapezoidalProfile_allocate(JNIEnv *env, jclass obj) {
  return jni_as_handle<trapezoidal>(new trapezoidal());
}