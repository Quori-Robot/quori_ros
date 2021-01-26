#include "param.hpp"

using namespace quori_face;
using namespace quori_face_node;

std::uint32_t quori_face_node::param(ros::NodeHandle &nh, const std::string &name, const std::uint32_t value)
{
  int res = 0;
  return nh.getParam(name, res) ? static_cast<std::uint32_t>(res) : value;
}

double quori_face_node::param(ros::NodeHandle &nh, const std::string &name, const double value)
{
  double res = 0;
  return nh.getParam(name, res) ? res : value;
}

bool quori_face_node::param(ros::NodeHandle &nh, const std::string &name, const bool value)
{
  bool res = false;
  return nh.getParam(name, res) ? res : value;
}

std::string quori_face_node::param(ros::NodeHandle &nh, const std::string &name, const std::string &value)
{
  std::string res;
  return nh.getParam(name, res) ? res : value;
}

SphericalCoordinate quori_face_node::param(ros::NodeHandle &nh, const std::string &name, const SphericalCoordinate &value)
{
  return SphericalCoordinate(
    param(nh, name + "/theta", value.theta),
    param(nh, name + "/psi", value.psi)
  );
}

TransformStaticParameters quori_face_node::param(ros::NodeHandle &nh, const std::string &name, const TransformStaticParameters &value)
{
  TransformStaticParameters ret;
  ret.R = param(nh, name + "/R", value.R);
  ret.r_m = param(nh, name + "/r_m", value.r_m);
  ret.r_o = param(nh, name + "/r_o", value.r_o);
  ret.h = param(nh, name + "/h", value.h);
  ret.L = param(nh, name + "/L", value.L);
  ret.epsilon = param(nh, name + "/epsilon", value.epsilon);
  ret.delta = param(nh, name + "/delta", value.delta);
  ret.screen_size = param(nh, name + "/screen_size", value.screen_size);
  return ret;
}