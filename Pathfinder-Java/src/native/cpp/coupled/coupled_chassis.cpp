#include "grpl_pathfinder_coupled_CoupledChassis.h"
#include "jnieigen.h"
#include "jnihandle.h"
#include "jniutil.h"
#include "coupled/jnicoupled.h"

#include <grpl/pf/coupled/chassis.h>
#include <grpl/pf/transmission/dc.h>

#include <memory>

using namespace grpl::pf::coupled;
using namespace grpl::pf::transmission;

// We use std::allocator and std::allocator_traits since we need to be able to reserve
// the memory (allocate) without constructing the chassis until more information is available
// (i.e. the native handles to left and right transmissions)
using allocator_t = std::allocator<chassis>;
using traits_t    = std::allocator_traits<allocator_t>;

allocator_t _ch_alloc;

static void wheelstate_to_java(JNIEnv *env, wheel_state &ws, jdoubleArray arr) {
  double tmp_state_arr[10];
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

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_coupled_CoupledChassis_allocateStub(JNIEnv *env, jclass claz) {
  return jni_as_handle<chassis>(traits_t::allocate(_ch_alloc, 1));
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledChassis_construct(JNIEnv *env, jclass claz,
                                                                             jlong handle, jlong leftHandle,
                                                                             jlong   rightHandle,
                                                                             jdouble wheelR, jdouble trackR,
                                                                             jdouble mass) {
  dc_transmission *trans_left  = jni_handle<dc_transmission>(env, leftHandle);
  dc_transmission *trans_right = jni_handle<dc_transmission>(env, rightHandle);
  traits_t::construct(_ch_alloc, jni_handle<chassis>(env, handle), *trans_left, *trans_right, wheelR, trackR,
                      mass);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledChassis_free(JNIEnv *env, jclass claz,
                                                                        jlong handle) {
  chassis *ch = jni_handle<chassis>(env, handle);
  traits_t::destroy(_ch_alloc, ch);
  traits_t::deallocate(_ch_alloc, ch, 1);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_coupled_CoupledChassis_linvelLimit(JNIEnv *env, jclass claz,
                                                                                  jlong        handle,
                                                                                  jdoubleArray config,
                                                                                  jdouble      curv) {
  configuration_state conf = eigen_adapt_jdoubleArray<configuration_state>(env, config);
  return jni_handle<chassis>(env, handle)->linear_vel_limit(conf, curv);
}

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_coupled_CoupledChassis_accLimit(
    JNIEnv *env, jclass claz, jlong handle, jdoubleArray config, jdouble curv, jdouble vel) {
  configuration_state       conf = eigen_adapt_jdoubleArray<configuration_state>(env, config);
  std::pair<double, double> ret  = jni_handle<chassis>(env, handle)->acceleration_limits(conf, curv, vel);

  double data[2]{ret.first, ret.second};

  jdoubleArray arr = env->NewDoubleArray(2);
  env->SetDoubleArrayRegion(arr, 0, 2, data);
  return arr;
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_coupled_CoupledChassis_splitNative(JNIEnv *env, jclass claz,
                                                                               jlong        handle,
                                                                               jdoubleArray centreArr,
                                                                               jdoubleArray leftArr,
                                                                               jdoubleArray rightArr) {
  std::pair<wheel_state, wheel_state> split =
      jni_handle<chassis>(env, handle)->split(jni_coupled_java_to_state(env, centreArr));

  wheelstate_to_java(env, split.first, leftArr);
  wheelstate_to_java(env, split.second, rightArr);
}