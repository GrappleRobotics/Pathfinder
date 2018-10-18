#pragma once

#include <jni.h>
#include <Eigen/Dense>

template <typename matrix_t>
inline matrix_t eigen_adapt_jdoubleArray(JNIEnv *env, jdoubleArray arr) {
  double * kin = env->GetDoubleArrayElements(arr, nullptr);
  matrix_t mat = Eigen::Map<matrix_t>(kin);
  env->ReleaseDoubleArrayElements(arr, kin, 0);
  return mat;
}

template <typename matrix_t>
inline matrix_t eigen_adapt_array(JNIEnv *env, double *arr) {
  matrix_t mat = Eigen::Map<matrix_t>(arr);
  return mat;
}

template <typename matrix_t>
inline jdoubleArray eigen_create_jdoubleArray(JNIEnv *env, matrix_t mat) {
  jdoubleArray arr = env->NewDoubleArray(mat.size());
  env->SetDoubleArrayRegion(arr, 0, mat.size(), mat.data());
  return arr;
}