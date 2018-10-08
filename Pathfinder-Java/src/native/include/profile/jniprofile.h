#pragma once

#include <Eigen/Dense>
#include "jnieigen.h"
#include "jniutil.h"

template <typename segment_t>
segment_t jni_array_to_native_segment(JNIEnv *env, jdouble t, jdoubleArray kinematics) {
  return segment_t{t, eigen_adapt_jdoubleArray<typename segment_t::kinematics_t>(env, kinematics)};
}

template <typename segment_t>
segment_t jni_java_to_native_segment(JNIEnv *env, jobject obj) {
  jdouble t = jni_get_double_field(env, obj, "time");

  jdoubleArray kin_array = jni_get_field<jdoubleArray>(env, obj, "kinematics", "[D");

  return jni_array_to_native_segment<segment_t>(env, t, kin_array);
}

template <typename segment_t>
jobject jni_segment_to_java(JNIEnv *env, segment_t seg) {
  jdoubleArray kin_arr = eigen_create_jdoubleArray(env, seg.kinematics);

  jobject j_seg =
      jni_construct(env, "grpl/pathfinder/profile/Profile$Segment", "(I)V", seg.kinematics.size());
  jni_set_double_field(env, j_seg, "time", seg.time);
  jni_set_field(env, j_seg, "kinematics", "[D", kin_arr);
  return j_seg;
}

template <typename segment_t>
jdoubleArray jni_segment_to_array(JNIEnv *env, segment_t seg) {
  return eigen_create_jdoubleArray(env, seg.kinematics);
}