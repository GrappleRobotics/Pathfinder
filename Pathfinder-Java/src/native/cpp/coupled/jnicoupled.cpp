#include "coupled/jnicoupled.h"

grpl::pf::coupled::state jni_coupled_java_to_state(JNIEnv *env, jdoubleArray arr) {
  double *                 els = env->GetDoubleArrayElements(arr, nullptr);
  grpl::pf::coupled::state s{els[0],
                             els[1],
                             els[2],
                             eigen_adapt_array<grpl::pf::coupled::configuration_state>(env, els + 3),
                             eigen_adapt_array<grpl::pf::coupled::kinematic_state>(env, els + 6),
                             els[9] > 0.5};
  env->ReleaseDoubleArrayElements(arr, els, 0);
  return s;
}