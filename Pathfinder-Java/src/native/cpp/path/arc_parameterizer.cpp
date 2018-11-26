#include "grpl_pathfinder_path_ArcParameterizer.h"
#include "jnieigen.h"
#include "jnihandle.h"
#include "jniutil.h"

#include <functional>
#include <vector>

#include <grpl/pf/path/arc_parameterizer.h>

using namespace grpl::pf::path;

using spline_container_t = std::vector<std::reference_wrapper<spline<2>>>;
using curve_container_t  = std::vector<arc_parameterizer::curve_t>;

class java_spline_wrapper : public spline<2> {
 public:
  using vector_t = typename spline::vector_t;

  java_spline_wrapper(JNIEnv *env, jobject obj) : _env(env) {
    _obj    = env->NewGlobalRef(obj);
    _posid  = jni_get_method_id(env, obj, "position", "(D)Lgrpl/pathfinder/path/Vec2;");
    _velid  = jni_get_method_id(env, obj, "derivative", "(D)Lgrpl/pathfinder/path/Vec2;");
    _rotid  = jni_get_method_id(env, obj, "rotation", "(D)Lgrpl/pathfinder/path/Vec2;");
    _curvid = jni_get_method_id(env, obj, "curvature", "(D)D");
    _vecxy  = jni_get_method_id(env, "grpl/pathfinder/path/Vec2", "xy", "()[D");
  }

  virtual ~java_spline_wrapper() { _env->DeleteGlobalRef(_obj); }

  vector_t position(double s) override {
    jobject      vec2 = _env->CallObjectMethod(_obj, _posid, s);
    jdoubleArray arr  = reinterpret_cast<jdoubleArray>(_env->CallObjectMethod(vec2, _vecxy));
    return eigen_adapt_jdoubleArray<vector_t>(_env, arr);
  }

  vector_t derivative(double s) override {
    jobject      vec2 = _env->CallObjectMethod(_obj, _velid, s);
    jdoubleArray arr  = reinterpret_cast<jdoubleArray>(_env->CallObjectMethod(vec2, _vecxy));
    return eigen_adapt_jdoubleArray<vector_t>(_env, arr);
  }

  vector_t rotation(double s) override {
    jobject      vec2 = _env->CallObjectMethod(_obj, _rotid, s);
    jdoubleArray arr  = reinterpret_cast<jdoubleArray>(_env->CallObjectMethod(vec2, _vecxy));
    return eigen_adapt_jdoubleArray<vector_t>(_env, arr);
  }

  double curvature(double s) override { return _env->CallDoubleMethod(_obj, _curvid, s); }

 private:
  JNIEnv *  _env;
  jobject   _obj;
  jmethodID _posid, _velid, _rotid, _curvid, _vecxy;
};

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_path_ArcParameterizer_allocate(JNIEnv *env, jclass claz) {
  return jni_as_handle<arc_parameterizer>(new arc_parameterizer());
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_path_ArcParameterizer_free(JNIEnv *env, jclass claz,
                                                                       jlong handle) {
  delete jni_handle<arc_parameterizer>(env, handle);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_path_ArcParameterizer_configure(JNIEnv *env, jclass claz,
                                                                            jlong   handle,
                                                                            jdouble maxArcLength,
                                                                            jdouble maxDeltaCurv) {
  jni_handle<arc_parameterizer>(env, handle)->configure(maxArcLength, maxDeltaCurv);
}

JNIEXPORT jlongArray JNICALL Java_grpl_pathfinder_path_ArcParameterizer_parameterize(JNIEnv *env, jclass claz,
                                                                                     jlong handle,
                                                                                     jlong bufferHandle) {
  spline_container_t *splines = jni_handle<spline_container_t>(env, bufferHandle);
  curve_container_t   curves;
  jni_handle<arc_parameterizer>(env, handle)
      ->parameterize(splines->begin(), splines->end(), std::back_inserter(curves), curves.max_size());

  return jni_copy_to_handles<arc_parameterizer::curve_t>(env, curves);
}

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_path_ArcParameterizer_acquireBuffer(JNIEnv *env, jclass claz) {
  return jni_as_handle<spline_container_t>(new spline_container_t());
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_path_ArcParameterizer_enqueueNative(JNIEnv *env, jclass claz,
                                                                                jlong bufferHandle,
                                                                                jlong splineHandle) {
  spline_container_t *splines = jni_handle<spline_container_t>(env, bufferHandle);

  spline<2> *spl = jni_handle<spline<2>>(env, splineHandle);
  splines->push_back(*spl);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_path_ArcParameterizer_enqueueAdapter(JNIEnv *env, jclass claz,
                                                                                 jlong   bufferHandle,
                                                                                 jobject javaSpline) {
  spline_container_t *splines = jni_handle<spline_container_t>(env, bufferHandle);
  spline<2> *         spl     = new java_spline_wrapper(env, javaSpline);
  splines->push_back(*spl);
}

JNIEXPORT void JNICALL Java_grpl_pathfinder_path_ArcParameterizer_releaseBuffer(JNIEnv *env, jclass claz,
                                                                                jlong bufferHandle) {
  delete jni_handle<spline_container_t>(env, bufferHandle);
}