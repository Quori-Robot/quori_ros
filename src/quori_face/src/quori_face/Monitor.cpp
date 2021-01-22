#include "quori_face/Monitor.hpp"

#include <GLFW/glfw3.h>

#include "trace.hpp"

using namespace quori_face;

std::vector<Monitor::Ptr> Monitor::getMonitors()
{
  int count = 0;
  GLFWmonitor **const monitors = QUORI_FACE_TRACE(glfwGetMonitors(&count));
  
  std::vector<Monitor::Ptr> ret;
  for (int i = 0; i < count; ++i)
  {
    ret.push_back(Ptr(new Monitor(monitors[i])));
  }

  return ret;
}

Monitor::Ptr Monitor::getPrimaryMonitor()
{
  return Ptr(new Monitor(glfwGetPrimaryMonitor()));
}

bool Monitor::isPrimary() const
{
  return QUORI_FACE_TRACE(glfwGetPrimaryMonitor() == handle_);
}

std::string Monitor::getName() const
{
  return QUORI_FACE_TRACE(glfwGetMonitorName(handle_));
}

GLFWmonitor *Monitor::getHandle() const
{
  return handle_;
}

Monitor::~Monitor()
{
}

Monitor::Monitor(GLFWmonitor *const handle)
  : handle_(handle)
{
}