#include "grpl_pathfinder_transmission_DcMotor.h"
#include "jnihandle.h"

#include <grpl/pf/transmission/dc.h>

using namespace grpl::pf::transmission;

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_transmission_DcMotor_allocate(
    JNIEnv *env, jclass claz, jdouble v_nom, jdouble free_speed, jdouble free_current,
    jdouble stall_current, jdouble stall_torque) {
  return jni_as_handle<dc_motor>(new dc_motor(v_nom, free_speed, free_current, stall_current, stall_torque));
}