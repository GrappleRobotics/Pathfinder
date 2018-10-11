#pragma once

#include <jni.h>
#include <vector>

jfieldID  jni_get_field_id(JNIEnv *env, jobject obj, const char *name, const char *sig);
jmethodID jni_get_method_id(JNIEnv *env, jobject obj, const char *name, const char *sig);
jmethodID jni_get_method_id(JNIEnv *env, const char *className, const char *name, const char *sig);

jdouble jni_get_double_field(JNIEnv *env, jobject obj, const char *name);

void jni_set_double_field(JNIEnv *env, jobject obj, const char *name, jdouble dbl);

template <typename type = jobject>
type jni_get_field(JNIEnv *env, jobject obj, const char *name, const char *sig) {
  return static_cast<type>(env->GetObjectField(obj, jni_get_field_id(env, obj, name, sig)));
}

template <typename type = jobject>
void jni_set_field(JNIEnv *env, jobject obj, const char *name, const char *sig, type data) {
  env->SetObjectField(obj, jni_get_field_id(env, obj, name, sig), static_cast<jobject>(data));
}

jobject jni_construct(JNIEnv *env, const char *cls_name, const char *sig, ...);

template <typename T>
jlongArray jni_copy_to_handles(JNIEnv *env, std::vector<T> &vec) {
  jlong *addr = new jlong[vec.size()];
  size_t i    = 0;
  for (typename std::vector<T>::iterator it = vec.begin(); it != vec.end(); ++it) {
    // Create a copy of T somewhere on the heap, since it's now Javas to deal with.
    addr[i++] = reinterpret_cast<jlong>(new T(*it));
  }

  jlongArray arr = env->NewLongArray(vec.size());
  env->SetLongArrayRegion(arr, 0, vec.size(), addr);
  delete[] addr;
  return arr;
}