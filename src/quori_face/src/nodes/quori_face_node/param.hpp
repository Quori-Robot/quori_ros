#ifndef _QUORI_FACE_NODE_PARAM_HPP_
#define _QUORI_FACE_NODE_PARAM_HPP_

#include <string>
#include <cstdint>
#include <ros/ros.h>

#include <quori_face/Vector2.hpp>
#include <quori_face/transform.hpp>

namespace quori_face_node
{
  std::uint32_t param(ros::NodeHandle &nh, const std::string &name, const std::uint32_t value);
  double param(ros::NodeHandle &nh, const std::string &name, const double value);

  template<typename T>
  quori_face::Vector2<T> param(ros::NodeHandle &nh, const std::string &name, const quori_face::Vector2<T> &value)
  {
    return quori_face::Vector2<T>(
      param(nh, name + "/x", value.x),
      param(nh, name + "/y", value.y)
    );
  }

  quori_face::SphericalCoordinate param(ros::NodeHandle &nh, const std::string &name, const quori_face::SphericalCoordinate &value);
  quori_face::TransformStaticParameters param(ros::NodeHandle &nh, const std::string &name, const quori_face::TransformStaticParameters &value);
}

#endif