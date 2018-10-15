#include "grpl_pathfinder_transmission_JavaDcTransmissionAdapter.h"
#include "jnihandle.h"
#include "jniutil.h"

#include <grpl/pf/transmission/dc.h>

using namespace grpl::pf::transmission;

class java_dc_transmission_adapter : public dc_transmission {
 public:
  java_dc_transmission_adapter(JNIEnv *env, jobject obj) : _env(env) {
    _obj       = env->NewGlobalRef(obj);
    _freespeed = jni_get_method_id(env, obj, "getFreeSpeed", "(D)D");
    _current   = jni_get_method_id(env, obj, "getCurrent", "(DD)D");
    _torque    = jni_get_method_id(env, obj, "getTorque", "(D)D");
    _freevolt  = jni_get_method_id(env, obj, "getFreeVoltage", "(D)D");
    _currvolt  = jni_get_method_id(env, obj, "getCurrentVoltage", "(D)D");
    _torcurr   = jni_get_method_id(env, obj, "getTorqueCurrent", "(D)D");
    _nomvolt   = jni_get_method_id(env, obj, "getNominalVoltage", "()D");
  }

  virtual ~java_dc_transmission_adapter() { _env->DeleteGlobalRef(_obj); }

  double get_free_speed(double voltage) const override {
    return _env->CallDoubleMethod(_obj, _freespeed, voltage);
  }

  double get_current(double voltage, double speed) const override {
    return _env->CallDoubleMethod(_obj, _current, voltage, speed);
  }

  double get_torque(double current) const override { return _env->CallDoubleMethod(_obj, _torque, current); }

  double get_free_voltage(double speed) const override {
    return _env->CallDoubleMethod(_obj, _freevolt, speed);
  }

  double get_current_voltage(double current) const override {
    return _env->CallDoubleMethod(_obj, _currvolt, current);
  }

  double get_torque_current(double torque) const override {
    return _env->CallDoubleMethod(_obj, _torcurr, torque);
  }

  double nominal_voltage() const override { return _env->CallDoubleMethod(_obj, _nomvolt); }

 private:
  JNIEnv *  _env;
  jobject   _obj;
  jmethodID _freespeed, _current, _torque;
  jmethodID _freevolt, _currvolt, _torcurr;
  jmethodID _nomvolt;
};

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_transmission_JavaDcTransmissionAdapter_allocate(
    JNIEnv *env, jclass claz, jobject javaobj) {
  return jni_as_handle<dc_transmission>(new java_dc_transmission_adapter(env, javaobj));
}