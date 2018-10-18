// See github issue #11

// #include "grpl_pathfinder_profile_JavaProfileAdapter.h"
// #include "jnihandle.h"
// #include "profile/jniprofile.h"

// #include <grpl/pf/profile/profile.h>

// using namespace grpl::pf::profile;

// class java_profile_adapter : public profile {
//  public:
//   java_profile_adapter(JNIEnv *env, jobject obj) : _env(env) {
//     _obj = env->NewGlobalRef(obj);
//     _limterm = env->CallIntMethod(obj, jni_get_method_id(env, obj, "getLimitedTerm", "()I"));

//     _calc = jni_get_method_id(env, obj, "calcFromNative", "([DDD])[D");
//   }

//   const size_t limited_term() const override { return _limterm; }

//   segment calculate(segment &last, double time) override {
//     jdoubleArray lastJava = jni_segment_to_array(_env, last);
//     jdouble lastTime = last.time;

//     jdoubleArray ret = static_cast<jdoubleArray>(_env->CallObjectMethod(_obj, _calc, lastJava, lastTime, time));
//     return jni_array_to_native_segment<segment>(_env, time, ret);
//   }

//  private:
//   JNIEnv *_env;
//   jobject _obj;
//   size_t  _limterm;

//   jmethodID _calc;
// };

// JNIEXPORT jlong JNICALL Java_grpl_pathfinder_profile_JavaProfileAdapter_allocate(JNIEnv *env, jclass claz,
//                                                                                  jobject jprofile) {
//   return jni_as_handle<profile>(new java_profile_adapter(env, jprofile));
// }