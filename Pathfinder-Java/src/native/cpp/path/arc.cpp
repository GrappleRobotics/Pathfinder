#include "grpl_pathfinder_path_Arc2d.h"
#include "jnieigen.h"
#include "jnihandle.h"

#include <grpl/pf/path/arc.h>

using namespace grpl::pf::path;

JNIEXPORT jlong JNICALL Java_grpl_pathfinder_path_Arc2d_allocate(JNIEnv *env, jclass claz,
                                                                 jdoubleArray startArr, jdoubleArray midArr,
                                                                 jdoubleArray endArr) {
  using arc_t = arc2d;

  arc_t::vector_t start = eigen_adapt_jdoubleArray<arc_t::vector_t>(env, startArr);
  arc_t::vector_t mid   = eigen_adapt_jdoubleArray<arc_t::vector_t>(env, midArr);
  arc_t::vector_t end   = eigen_adapt_jdoubleArray<arc_t::vector_t>(env, endArr);

  return jni_as_handle<arc_t>(new arc_t(start, mid, end));
}