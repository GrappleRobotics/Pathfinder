#pragma once

#include <Eigen/Dense>
#include "grpl/pf/coupled/state.h"
#include "jnieigen.h"
#include "jniutil.h"

grpl::pf::coupled::state jni_coupled_java_to_state(JNIEnv *env, jdoubleArray arr);