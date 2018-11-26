#include "profile/jniprofile.h"

grpl::pf::profile::state jni_array_to_native_state(JNIEnv *env, jdouble t, jdoubleArray kinematics) {
  return grpl::pf::profile::state{t, eigen_adapt_jdoubleArray<grpl::pf::profile::kinematic_state>(env, kinematics)};
}

grpl::pf::profile::state jni_java_to_native_state(JNIEnv *env, jobject obj) {
  jdouble t = jni_get_double_field(env, obj, "time");

  jdoubleArray kin_array = jni_get_field<jdoubleArray>(env, obj, "kinematics", "[D");

  return jni_array_to_native_state(env, t, kin_array);
}

jobject jni_state_to_java(JNIEnv *env, grpl::pf::profile::state st) {
  jdoubleArray kin_arr = eigen_create_jdoubleArray(env, st.kinematics);

  jobject j_st =
      jni_construct(env, "grpl/pathfinder/profile/Profile$State", "(I)V", st.kinematics.size());
  jni_set_double_field(env, j_st, "time", st.time);
  jni_set_field(env, j_st, "kinematics", "[D", kin_arr);
  return j_st;
}

jdoubleArray jni_state_to_array(JNIEnv *env, grpl::pf::profile::state st) {
  return eigen_create_jdoubleArray(env, st.kinematics);
}