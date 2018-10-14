#include "grpl_pathfinder_transmission_AbstractDcTransmission.h"
#include "jnihandle.h"

#include <grpl/pf/transmission/dc.h>

using namespace grpl::pf::transmission;

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_transmission_AbstractDcTransmission_freeSpeed(
    JNIEnv *env, jclass claz, jlong handle, jdouble voltage) {
  return jni_handle<dc_transmission>(env, handle)->get_free_speed(voltage);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_transmission_AbstractDcTransmission_current(
    JNIEnv *env, jclass claz, jlong handle, jdouble voltage, jdouble speed) {
  return jni_handle<dc_transmission>(env, handle)->get_current(voltage, speed);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_transmission_AbstractDcTransmission_torque(JNIEnv *env,
                                                                                          jclass  claz,
                                                                                          jlong   handle,
                                                                                          jdouble current) {
  return jni_handle<dc_transmission>(env, handle)->get_torque(current);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_transmission_AbstractDcTransmission_freeVoltage(
    JNIEnv *env, jclass claz, jlong handle, jdouble speed) {
  return jni_handle<dc_transmission>(env, handle)->get_free_voltage(speed);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_transmission_AbstractDcTransmission_currentVoltage(
    JNIEnv *env, jclass claz, jlong handle, jdouble current) {
  return jni_handle<dc_transmission>(env, handle)->get_current_voltage(current);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_transmission_AbstractDcTransmission_torqueCurrent(
    JNIEnv *env, jclass claz, jlong handle, jdouble torque) {
  return jni_handle<dc_transmission>(env, handle)->get_torque_current(torque);
}

JNIEXPORT jdouble JNICALL Java_grpl_pathfinder_transmission_AbstractDcTransmission_nominalVoltage(
    JNIEnv *env, jclass claz, jlong handle) {
  return jni_handle<dc_transmission>(env, handle)->nominal_voltage();
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_transmission_AbstractDcTransmission_free(JNIEnv *env, jclass claz,
                                                                                     jlong handle) {
  delete jni_handle<dc_transmission>(env, handle);
}
