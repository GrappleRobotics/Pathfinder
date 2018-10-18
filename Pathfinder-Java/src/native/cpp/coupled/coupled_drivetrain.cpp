#include "grpl_pathfinder_coupled_CoupledDrivetrain.h"
#include "jnieigen.h"
#include "jnihandle.h"
#include "jniutil.h"

#include <grpl/pf/coupled/drivetrain.h>
#include <grpl/pf/path/curve.h>
#include <grpl/pf/profile/profile.h>

#include <memory>
#include <vector>

using namespace grpl::pf::coupled;
using namespace grpl::pf::path;
using namespace grpl::pf::profile;

using curve_container_t = std::vector<std::reference_wrapper<curve<2>>>;

static double tmp_state_arr[10];

static state java_to_state(JNIEnv *env, jdoubleArray arr) {
  double *els = env->GetDoubleArrayElements(arr, nullptr);
  state   s{els[0],
          els[1],
          els[2],
          eigen_adapt_array<configuration>(env, els + 3),
          eigen_adapt_array<kinematics>(env, els + 6),
          els[9] > 0.5};
  env->ReleaseDoubleArrayElements(arr, els, 0);
  return s;
}

static jdoubleArray state_to_java(JNIEnv *env, state &st) {
  jdoubleArray nArr = env->NewDoubleArray(10);
  tmp_state_arr[0]  = st.time;
  tmp_state_arr[1]  = st.curvature;
  tmp_state_arr[2]  = st.dcurvature;
  for (int i = 0; i < 3; i++) {
    tmp_state_arr[3 + i] = st.configuration[i];
    tmp_state_arr[6 + i] = st.kinematics[i];
  }
  tmp_state_arr[9] = st.finished ? 1.0 : 0.0;
  env->SetDoubleArrayRegion(nArr, 0, 10, tmp_state_arr);
  return nArr;
}

static void wheelstate_to_java(JNIEnv *env, wheel_state &ws, jdoubleArray arr) {
  tmp_state_arr[0] = ws.time;
  tmp_state_arr[1] = ws.position[0];
  tmp_state_arr[2] = ws.position[1];
  for (int i = 0; i < 3; i++) {
    tmp_state_arr[3 + i] = ws.kinematics[i];
  }
  tmp_state_arr[6] = ws.voltage;
  tmp_state_arr[7] = ws.current;
  tmp_state_arr[8] = ws.finished ? 1.0 : 0.0;
  env->SetDoubleArrayRegion(arr, 0, 9, tmp_state_arr);
}

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_coupled_CoupledDrivetrain_allocate(JNIEnv *env, jclass claz,
                                                                                jlong chassisHandle) {
  return jni_as_handle<drivetrain>(new drivetrain(*jni_handle<chassis>(env, chassisHandle)));
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledDrivetrain_free(JNIEnv *env, jclass claz,
                                                                           jlong handle) {
  delete jni_handle<drivetrain>(env, handle);
}

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_coupled_CoupledDrivetrain_generateNative(
    JNIEnv *env, jclass claz, jlong handle, jlong buff, jlong profile_handle, jdoubleArray stateArr,
    jdouble time) {
  curve_container_t *curves = jni_handle<curve_container_t>(env, buff);
  profile *          prof   = jni_handle<profile>(env, profile_handle);

  state s = java_to_state(env, stateArr);
  state n = jni_handle<drivetrain>(env, handle)->generate(curves->begin(), curves->end(), *prof, s, time);

  return state_to_java(env, n);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledDrivetrain_splitNative(JNIEnv *env, jclass claz,
                                                                                  jlong        handle,
                                                                                  jdoubleArray centreArr,
                                                                                  jdoubleArray leftArr,
                                                                                  jdoubleArray rightArr) {
  std::pair<wheel_state, wheel_state> split =
      jni_handle<drivetrain>(env, handle)->split(java_to_state(env, centreArr));

  wheelstate_to_java(env, split.first, leftArr);
  wheelstate_to_java(env, split.second, rightArr);
}

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_coupled_CoupledDrivetrain_acquireBuffer(JNIEnv *env,
                                                                                     jclass  claz) {
  return jni_as_handle<curve_container_t>(new curve_container_t());
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledDrivetrain_enqueueNative(JNIEnv *env, jclass claz,
                                                                                    jlong buff,
                                                                                    jlong curveHandle) {
  curve_container_t *curves = jni_handle<curve_container_t>(env, buff);

  curve<2> *cur = jni_handle<curve<2>>(env, curveHandle);
  curves->push_back(*cur);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledDrivetrain_enqueueAdapter(JNIEnv *env, jclass claz,
                                                                                     jlong   buff,
                                                                                     jobject curve) {
  // TODO
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledDrivetrain_releaseBuffer(JNIEnv *env, jclass claz,
                                                                                    jlong buff) {
  delete jni_handle<curve_container_t>(env, buff);
}