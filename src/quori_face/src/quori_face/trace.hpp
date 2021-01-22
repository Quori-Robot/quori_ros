#ifndef _QUORI_FACE_TRACE_HPP_
#define _QUORI_FACE_TRACE_HPP_

#include <iostream>
#include <cstring>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define __COMPACT_PRETTY_FUNCTION__ quori_face::computeMethodName(__FUNCTION__, __PRETTY_FUNCTION__).c_str()
#ifdef QUORI_FACE_TRACING
#define QUORI_FACE_TRACE(stmt) \
  ({ \
  std::cout << __FILENAME__ << ":" << __LINE__ << " (" << __COMPACT_PRETTY_FUNCTION__ << "): " << #stmt << std::endl; \
  stmt; \
  }) 
#else
#define QUORI_FACE_TRACE(stmt) \
  ({ \
  stmt; \
  }) 
#endif

namespace quori_face
{
  std::string computeMethodName(const std::string& function, const std::string& prettyFunction);
}

#endif
