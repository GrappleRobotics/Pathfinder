#pragma once

#include <jni.h>

jfieldID jni_get_handle_field(JNIEnv *env, jobject obj);

template <typename T>
T *jni_get_handle(JNIEnv *env, jobject obj) {
  jlong handle = env->GetLongField(obj, jni_get_handle_field(env, obj));

  T *t = reinterpret_cast<T *>(handle);
  if (t == nullptr) {
    const char *exbuff = "This object is destroyed!";
    env->ThrowNew(env->FindClass("grpl/pathfinder/util/DestroyException"), exbuff);
  }
  return t;
}

template <typename T>
void jni_set_handle(JNIEnv *env, jobject obj, T *t) {
  jlong handle = reinterpret_cast<jlong>(t);
  env->SetLongField(obj, jni_get_handle_field(env, obj), handle);
}

template <typename T>
jlong jni_as_handle(T *t) {
  return reinterpret_cast<jlong>(t);
}