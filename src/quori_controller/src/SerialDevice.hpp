#pragma once

#include <string>
#include <cstdint>
#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/read.hpp>

#include <vector>
#include <ros/time.h>

#include <optional>
#include "Csv.hpp"

#include "message.hpp"

#include "Joint.hpp"

namespace quori_controller
{
  class SerialDevice
  {
  public:
    struct InitializationInfo
    {
      struct JointInfo
      {
        std::string name;
        Joint::Mode mode;
      };

      std::vector<JointInfo> joints;
    };

    typedef std::shared_ptr<SerialDevice> Ptr;
    typedef std::shared_ptr<const SerialDevice> ConstPtr;

    ~SerialDevice();

    const std::string &getName() const;

    static Ptr open(boost::asio::io_service &context, const std::string &path);

    InitializationInfo initialize();

    void setPositions(const double *const positions, const std::size_t length);
    void set(const double *const positions, const double *const velocities, const std::size_t length);

    void getState(quori::message::States &states);

    void attachSetPositionsCsv(const Csv::Ptr &csv);
    Csv::Ptr detachSetPositionsCsv();

  private:
    SerialDevice(boost::asio::posix::stream_descriptor &&fd, const std::string &name);

    void read();

    template<typename T>
    size_t write(const T &msg)
    {
      boost::system::error_code ec;

      const size_t res = boost::asio::write(fd_, boost::asio::buffer(reinterpret_cast<const uint8_t *>(&msg), sizeof(msg)), boost::asio::transfer_all(), ec);

      if (ec)
      {
        throw boost::system::system_error(ec);
      }

      return res;
    }


    boost::asio::posix::stream_descriptor fd_;
    std::string name_;

    std::uint8_t numJoints_;

    std::size_t stateBufferSize_;
    std::uint8_t *stateBuffer_;

    std::vector<std::uint8_t> unprocessed_;

    Csv::Ptr set_positions_csv_;
  };
}