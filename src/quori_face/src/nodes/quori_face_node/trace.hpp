#ifndef _QUORI_FACE_NODE_TRACE_HPP_
#define _QUORI_FACE_NODE_TRACE_HPP_

#include <iostream>
#include <cstring>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define __COMPACT_PRETTY_FUNCTION__ quori_face_node::computeMethodName(__FUNCTION__, __PRETTY_FUNCTION__).c_str()
#ifdef QUORI_FACE_NODE_TRACING
#define QUORI_FACE_NODE_TRACE(stmt) \
  ({ \
  std::cout << __FILENAME__ << ":" << __LINE__ << " (" << __COMPACT_PRETTY_FUNCTION__ << "): " << #stmt << std::endl; \
  stmt; \
  }) 
#else
#define QUORI_FACE_NODE_TRACE(stmt) \
  ({ \
  stmt; \
  }) 
#endif

namespace quori_face_node
{
  std::string computeMethodName(const std::string& function, const std::string& prettyFunction);
}

#endif
