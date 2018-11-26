#include "grpl_pathfinder_coupled_CoupledCausalTrajGen.h"
#include "jnieigen.h"
#include "jnihandle.h"
#include "jniutil.h"
#include "coupled/jnicoupled.h"

#include <grpl/pf/coupled/causal_trajectory_generator.h>
#include <grpl/pf/path/curve.h>
#include <grpl/pf/profile/profile.h>

#include <memory>
#include <vector>

using namespace grpl::pf;

using curve_container_t = std::vector<std::reference_wrapper<path::curve<2>>>;

static jdoubleArray state_to_java(JNIEnv *env, coupled::state &st) {
  double tmp_state_arr[10];
  jdoubleArray nArr = env->NewDoubleArray(10);
  tmp_state_arr[0]  = st.time;
  tmp_state_arr[1]  = st.curvature;
  tmp_state_arr[2]  = st.dcurvature;
  for (int i = 0; i < 3; i++) {
    tmp_state_arr[3 + i] = st.config[i];
    tmp_state_arr[6 + i] = st.kinematics[i];
  }
  tmp_state_arr[9] = st.finished ? 1.0 : 0.0;
  env->SetDoubleArrayRegion(nArr, 0, 10, tmp_state_arr);
  return nArr;
}

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_coupled_CoupledCausalTrajGen_allocate(JNIEnv *env, jclass claz) {
  return jni_as_handle<coupled::causal_trajectory_generator>(new coupled::causal_trajectory_generator());
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledCausalTrajGen_free(JNIEnv *env, jclass claz,
                                                                              jlong handle) {
  delete jni_handle<coupled::causal_trajectory_generator>(env, handle);
}

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_coupled_CoupledCausalTrajGen_generateNative(
    JNIEnv *env, jclass claz, jlong handle, jlong chassisHandle, jlong buff, jlong profile_handle,
    jdoubleArray stateArr, jdouble time) {
  curve_container_t *curves = jni_handle<curve_container_t>(env, buff);

  profile::profile *prof = jni_handle<profile::profile>(env, profile_handle);
  coupled::chassis *ch   = jni_handle<coupled::chassis>(env, chassisHandle);

  coupled::state s = jni_coupled_java_to_state(env, stateArr);
  coupled::state n = jni_handle<coupled::causal_trajectory_generator>(env, handle)
                ->generate(*ch, curves->begin(), curves->end(), *prof, s, time);

  return state_to_java(env, n);
}

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_coupled_CoupledCausalTrajGen_acquireBuffer(JNIEnv *env,
                                                                                        jclass  claz) {
  return jni_as_handle<curve_container_t>(new curve_container_t());
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledCausalTrajGen_enqueueNative(JNIEnv *env,
                                                                                       jclass  claz,
                                                                                       jlong   buff,
                                                                                       jlong   curveHandle) {
  curve_container_t *curves = jni_handle<curve_container_t>(env, buff);

  path::curve<2> *cur = jni_handle<path::curve<2>>(env, curveHandle);
  curves->push_back(*cur);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledCausalTrajGen_enqueueAdapter(JNIEnv *env,
                                                                                        jclass  claz,
                                                                                        jlong   buff,
                                                                                        jobject curve) {
  // TODO
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledCausalTrajGen_releaseBuffer(JNIEnv *env,
                                                                                       jclass  claz,
                                                                                       jlong   buff) {
  delete jni_handle<curve_container_t>(env, buff);
}