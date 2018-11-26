#pragma once

#include <Eigen/Dense>
#include "jnieigen.h"
#include "jniutil.h"
#include "grpl/pf/profile/profile.h"

grpl::pf::profile::state jni_array_to_native_state(JNIEnv *env, jdouble t, jdoubleArray kinematics);

grpl::pf::profile::state jni_java_to_native_state(JNIEnv *env, jobject obj);

jobject jni_state_to_java(JNIEnv *env, grpl::pf::profile::state st);

jdoubleArray jni_state_to_array(JNIEnv *env, grpl::pf::profile::state st);