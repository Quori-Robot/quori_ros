#include "SerialDevice.hpp"

#include <fcntl.h>
#include <termios.h>

#include <boost/asio/write.hpp>
#include <boost/asio/read.hpp>

#include <iostream>
#include <chrono>

namespace
{
  std::array<std::uint8_t, 4> INITIALIZE { 1, 0, 0, 0, };
  std::array<std::uint8_t, 4> SET_POSITIONS { 2, 0, 0, 0 };
}

template<typename T>
std::ostream &operator <<(std::ostream &o, const std::vector<T> &data)
{
  for (auto it = data.cbegin(); it != data.cend(); ++it)
  {
    o << static_cast<int>(*it) << " ";
  }
  return o;
}

using namespace quori_controller;

SerialDevice::~SerialDevice()
{
  fd_.close();
  delete[] stateBuffer_;
}

// 1538908420

SerialDevice::Ptr SerialDevice::open(boost::asio::io_service &context, const std::string &path)
{
  const int fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK);
  if (fd < 0)
  {
    throw std::runtime_error(strerror(errno));
  }

  // std::cout << "Open " << path << std::endl;

  struct termios settings;
  tcgetattr(fd, &settings);

  cfsetispeed(&settings, 115200); /* baud rate */
  cfsetospeed(&settings, 115200); /* baud rate */
  settings.c_cflag &= ~PARENB; /* no parity */
  settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
  settings.c_cflag &= ~CSIZE;
  settings.c_cflag |= CS8; /* 8 bits */
  settings.c_cflag &= ~CRTSCTS;
  settings.c_cflag |= CREAD | CLOCAL;
  settings.c_iflag &= ~(IXON | IXOFF | IXANY);
  settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* canonical mode */
  settings.c_oflag &= ~OPOST; /* raw output */

  tcsetattr(fd, TCSANOW, &settings); /* apply the settings */
  if (tcsetattr(fd, TCSAFLUSH, &settings) < 0) {
    perror("init_serialport: Couldn't set term attributes");
    return nullptr;
  }

  return std::shared_ptr<SerialDevice>(new SerialDevice(boost::asio::posix::stream_descriptor(context, fd), path));
}

SerialDevice::SerialDevice(boost::asio::posix::stream_descriptor &&fd, const std::string &name)
  : fd_(std::move(fd))
  , name_(name)
  , numJoints_(0)
  , stateBufferSize_(0)
  , stateBuffer_(nullptr)
{
  
}

const std::string &SerialDevice::getName() const
{
  return name_;
}

SerialDevice::InitializationInfo SerialDevice::initialize()
{
  boost::system::error_code ec;


  usleep(10000UL);
  quori::message::Initialize initialize;


  std::cout << "Initialize " << name_ << std::endl;

  size_t count = write(initialize);


  bool exit = false;
  while (unprocessed_.size() < sizeof(quori::message::Initialized))
  {
    usleep(10000UL);
    read();

    // std::cout << unprocessed_.size() << std::endl;
  }


  const quori::message::Initialized *const msg = reinterpret_cast<const quori::message::Initialized *>(unprocessed_.data());

  InitializationInfo ret;
  if (strlen(msg->_0) > 0) ret.joints.push_back(InitializationInfo::JointInfo {
    msg->_0,
    static_cast<Joint::Mode>(msg->modes[0])
  });
  if (strlen(msg->_1) > 0) ret.joints.push_back(InitializationInfo::JointInfo {
    msg->_1,
    static_cast<Joint::Mode>(msg->modes[1])
  });
  if (strlen(msg->_2) > 0) ret.joints.push_back(InitializationInfo::JointInfo {
    msg->_2,
    static_cast<Joint::Mode>(msg->modes[2])
  });
  
  unprocessed_.erase(unprocessed_.begin(), unprocessed_.begin() + sizeof(quori::message::Initialized));



  return ret;
}

void SerialDevice::setPositions(const double *const positions, const std::size_t length)
{
  quori::message::SetPositions msg;
  msg.positions[0] = length > 0 ? positions[0] : 0;
  msg.positions[1] = length > 1 ? positions[1] : 0;
  msg.positions[2] = length > 2 ? positions[2] : 0;

  write(msg);

  while (unprocessed_.size() < sizeof(quori::message::SetPositionsRes))
  {
    read();
  }

  auto res = *reinterpret_cast<const quori::message::SetPositionsRes *>(unprocessed_.data());
  unprocessed_.erase(unprocessed_.begin(), unprocessed_.begin() + sizeof(quori::message::SetPositionsRes));
}

void SerialDevice::set(const double *const positions, const double *const velocities, const std::size_t length)
{
  quori::message::Set msg;
  msg.positions[0] = length > 0 ? positions[0] : 0;
  msg.positions[1] = length > 1 ? positions[1] : 0;
  msg.positions[2] = length > 2 ? positions[2] : 0;
  msg.velocities[0] = length > 0 ? velocities[0] : 0;
  msg.velocities[1] = length > 1 ? velocities[1] : 0;
  msg.velocities[2] = length > 2 ? velocities[2] : 0;

  write(msg);
  
  while (unprocessed_.size() < sizeof(quori::message::SetPositionsRes))
  {
    read();
  }
  auto res = *reinterpret_cast<const quori::message::SetPositionsRes *>(unprocessed_.data());
  unprocessed_.erase(unprocessed_.begin(), unprocessed_.begin() + sizeof(quori::message::SetPositionsRes));
}

void SerialDevice::getState(quori::message::States &state)
{
  quori::message::GetStates msg;
  write(msg);
  const auto start = std::chrono::system_clock::now();

  while (unprocessed_.size() < sizeof(quori::message::States))
  {
    read();
  }
  const auto end = std::chrono::system_clock::now();
  


  state = *reinterpret_cast<const quori::message::States *>(unprocessed_.data());
  unprocessed_.erase(unprocessed_.begin(), unprocessed_.begin() + sizeof(quori::message::States));
}

void SerialDevice::read()
{
  std::array<std::uint8_t, 256> buffer;
  boost::system::error_code ec;
  size_t count = boost::asio::read(fd_, boost::asio::buffer(buffer), boost::asio::transfer_at_least(1), ec);
  if (count == 0) return;

  unprocessed_.insert(unprocessed_.end(), buffer.begin(), buffer.begin() + count);

  while (unprocessed_.size() >= sizeof(quori::message::Log) && static_cast<quori::message::Type>(unprocessed_[0]) == quori::message::Type::Log)
  {
    const auto msg = reinterpret_cast<const quori::message::Log *>(unprocessed_.data());
    std::cout << name_ << ": " << msg->message << std::endl;
    unprocessed_.erase(unprocessed_.begin(), unprocessed_.begin() + sizeof(quori::message::Log));
  }
}