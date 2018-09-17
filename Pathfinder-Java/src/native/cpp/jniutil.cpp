#include "jniutil.h"

jfieldID jni_get_field_id(JNIEnv *env, jobject obj, const char *name, const char *sig) {
  return env->GetFieldID(env->GetObjectClass(obj), name, sig);
}

jmethodID jni_get_method_id(JNIEnv *env, jobject obj, const char *name, const char *sig) {
  return env->GetMethodID(env->GetObjectClass(obj), name, sig);
}

jmethodID jni_get_method_id(JNIEnv *env, const char *className, const char *name, const char *sig) {
  return env->GetMethodID(env->FindClass(className), name, sig);
}

jdouble jni_get_double_field(JNIEnv *env, jobject obj, const char *name) {
  return env->GetDoubleField(obj, jni_get_field_id(env, obj, name, "D"));
}

void jni_set_double_field(JNIEnv *env, jobject obj, const char *name, jdouble dbl) {
  env->SetDoubleField(obj, jni_get_field_id(env, obj, name, "D"), dbl);
}

jobject jni_construct(JNIEnv *env, const char *cls_name, const char *sig, ...) {
  jclass    clazz       = env->FindClass(cls_name);
  jmethodID constructor = env->GetMethodID(clazz, "<init>", sig);

  va_list args;
  va_start(args, sig);
  jobject result = env->NewObjectV(clazz, constructor, args);
  va_end(args);
  // jobject result = env->NewObject(clazz, constructor);
  return result;
}