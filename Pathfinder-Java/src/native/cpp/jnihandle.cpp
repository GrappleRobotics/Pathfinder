#include "jnihandle.h"
#include "jniutil.h"

jfieldID jni_get_handle_field(JNIEnv *env, jobject obj) {
  return jni_get_field_id(env, obj, "_handle", "J");
}