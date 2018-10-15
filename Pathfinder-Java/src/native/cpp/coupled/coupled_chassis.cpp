#include "grpl_pathfinder_coupled_CoupledChassis.h"
#include "jnieigen.h"
#include "jnihandle.h"
#include "jniutil.h"

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
  configuration conf = eigen_adapt_jdoubleArray<configuration>(env, config);
  return jni_handle<chassis>(env, handle)->linear_vel_limit(conf, curv);
}

JNIEXPORT jdoubleArray JNICALL Java_grpl_pathfinder_coupled_CoupledChassis_accLimit(
    JNIEnv *env, jclass claz, jlong handle, jdoubleArray config, jdouble curv, jdouble vel) {
  configuration             conf = eigen_adapt_jdoubleArray<configuration>(env, config);
  std::pair<double, double> ret  = jni_handle<chassis>(env, handle)->acceleration_limits(conf, curv, vel);

  double data[2]{ret.first, ret.second};

  jdoubleArray arr = env->NewDoubleArray(2);
  env->SetDoubleArrayRegion(arr, 0, 2, data);
  return arr;
}
