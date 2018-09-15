#pragma once

#include <jni.h>

jfieldID jni_get_field_id(JNIEnv *env, jobject obj, const char *name, const char *sig);

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