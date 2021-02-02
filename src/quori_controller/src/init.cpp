// This program creates the udev rules necessary for quori's microcontrollers.
// It does this by first scanning for Arduinos, then connecting to each arduino
// and reading its published joints. We record the serial numbers of each device
// and write out the correct udev rules.

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <boost/filesystem.hpp>
#include <unordered_map>

#include <fstream>

#include "SerialDevice.hpp"
#include "message.hpp"



using namespace boost::filesystem;

using namespace quori_controller;

std::string run(const path &program, const std::vector<std::string> &args)
{
  const char **argv = new const char *[args.size() + 2];

  size_t index = 0;
  argv[index++] = program.c_str();
  for (auto it = args.cbegin(); it != args.cend(); ++it, ++index)
  {
    argv[index] = it->c_str();
  }
  argv[index] = NULL;

  int pipefd[2];
  
  if (pipe(pipefd) < 0) throw std::runtime_error(strerror(errno));
  

  const pid_t pid = fork();
  if(pid < 0) throw std::runtime_error(strerror(errno));

  if (pid == 0) {
    dup2(pipefd[1], STDOUT_FILENO);
    close(pipefd[0]);
    close(pipefd[1]);

    execvp(program.c_str(), const_cast<char *const *>(argv));
    _exit(1);
  }

  delete[] argv;

  close(pipefd[1]);

  std::string out;
  for (;;)
  {
    char buf[1024];
    int count = read(pipefd[0], buf, sizeof buf);
    if (count <= 0) break;

    out += std::string(buf, buf + count);
  }

  int status;
  wait(&status);

  if (status != 0) {
    std::ostringstream o;
    o << program << " returned " << status;
    throw std::runtime_error(o.str());
  }

  return out;
}

struct DeviceInfo
{
  path device_path;
  std::string id_vendor;
  std::string id_serial;
};

std::ostream &operator <<(std::ostream &o, const DeviceInfo &v)
{
  return o << v.device_path << " (id_serial: " << v.id_serial << ", id_vendor: " << v.id_vendor << ")";
}

std::string extract_value(const std::string &text, const std::string &key)
{

  const std::size_t index = text.find(key);
  if (index == std::string::npos)
  {
    std::ostringstream o;
    o << "Failed to find key \"" << key << "\"";
    throw std::runtime_error(o.str());
  }
  const std::size_t begin = index + key.size();
  const std::size_t end = text.find("\n", begin);
  return text.substr(begin, end - begin);
}

DeviceInfo device_info(const path &device_path)
{
  const std::string output = run("/usr/bin/udevadm", { "info", device_path.string() });
  DeviceInfo ret;
  ret.device_path = device_path;
  ret.id_vendor = extract_value(output, "ID_VENDOR=");
  ret.id_serial = extract_value(output, "ID_SERIAL_SHORT=");
  return ret;
}

template<typename T>
std::ostream &operator <<(std::ostream &o, const std::vector<T> &ts)
{
  for (const auto &t : ts)
  {
    o << "  " << t << std::endl;
  }
  return o;
}

template<typename K, typename V>
std::ostream &operator <<(std::ostream &o, const std::unordered_map<K, V> &ts)
{
  for (const auto &t : ts)
  {
    o << "  " << t.first << ": " << t.second << std::endl;
  }
  return o;
}

enum class Microcontroller : std::uint8_t
{
  Unknown,
  LeftArm,
  RightArm,
  Waist,
  Base
};

namespace std
{
  template<>
  struct hash<Microcontroller>
  {
    size_t operator() (const Microcontroller &microcontroller) const noexcept
    {
      return static_cast<std::uint8_t>(microcontroller);
    }
  };
}

std::ostream &operator <<(std::ostream &o, const Microcontroller v)
{
  switch (v)
  {
    case Microcontroller::Unknown: return o << "Unknown";
    case Microcontroller::LeftArm: return o << "Left Arm";
    case Microcontroller::RightArm: return o << "Right Arm";
    case Microcontroller::Waist: return o << "Waist";
    case Microcontroller::Base: return o << "Base";
  }
}

std::string microcontroller_filename(const Microcontroller v)
{
  switch (v)
  {
    case Microcontroller::Unknown: return "unknown";
    case Microcontroller::LeftArm: return "left_arm";
    case Microcontroller::RightArm: return "right_arm";
    case Microcontroller::Waist: return "waist";
    case Microcontroller::Base: return "base";
  }
}

std::string udev_rule(const Microcontroller microcontroller, const std::string &serial)
{
  std::ostringstream o;
  o << "KERNEL==\"ttyACM[0-9]*\", SUBSYSTEM==\"tty\", ATTRS{serial}==\"" << serial << "\", SYMLINK+=\"quori/" << microcontroller_filename(microcontroller) << "\", MODE=\"0666\"" << std::endl;
  return o.str();
}

int main(int argc, char *argv[])
{
  std::cout << "sizeof " << sizeof(quori::message::Initialize) << std::endl;
  // Iterate through the /dev directory, finding devices of the form "ttyACM*"
  std::vector<DeviceInfo> teensys;
  std::unordered_map<Microcontroller, std::string> serials;

  for (directory_iterator it("/dev"); it != directory_iterator(); ++it)
  {
    if (it->path().filename().string().find("ttyACM") == std::string::npos) continue;

    // Each USB device has a VENDOR property we can introspect on.
    // We only want Arduino Teensys
    const DeviceInfo info = device_info(it->path());

    if (info.id_vendor == "STMicroelectronics")
    {
      serials.insert({ Microcontroller::Base, info.id_serial });
      continue;
    }
    
    if (info.id_vendor != "Teensyduino") continue;
    
    teensys.push_back(info);
  }

  std::cout << "Found " << teensys.size() << " microcontrollers:" << std::endl << teensys;

  boost::asio::io_service service;



  std::cout << "Discovering mappings... ";
  std::cout.flush();
  // Connect to each teensy and determine which joints it has
  for (const auto &teensy : teensys)
  {
    // std::cout << teensy.device_path << std::endl;
    const auto serial_device = SerialDevice::open(service, teensy.device_path.string());
    const auto info = serial_device->initialize();

    Microcontroller microcontroller = Microcontroller::Unknown;
    const std::string &joint = info.joints[0].name;
    std::cout << joint << std::endl;
    if (joint.find("left_arm") != std::string::npos)
    {
      microcontroller = Microcontroller::LeftArm;
    }
    else if (joint.find("right_arm") != std::string::npos)
    {
      microcontroller = Microcontroller::RightArm;
    }
    else if (joint.find("waist") != std::string::npos)
    {
      microcontroller = Microcontroller::Waist;
    }
    else if (joint.find("base") != std::string::npos)
    {
      microcontroller = Microcontroller::Base;
    }

    if (microcontroller == Microcontroller::Unknown)
    {
      std::ostringstream o;
      o << "Unable to determine which quori microcontroller \"" << teensy << "\" is";
      throw std::runtime_error(o.str());
    }

    serials.insert({ microcontroller, teensy.id_serial });
  }

  std::cout << "done!" << std::endl;



  const auto left_arm_it = serials.find(Microcontroller::LeftArm);
  if (left_arm_it == serials.end())
  {
    std::ostringstream o;
    o << "Didn't find Quori's Left Arm";
    throw std::runtime_error(o.str());
  }

  const auto right_arm_it = serials.find(Microcontroller::RightArm);
  if (right_arm_it == serials.end())
  {
    std::ostringstream o;
    o << "Didn't find Quori's Right Arm";
    throw std::runtime_error(o.str());
  }

  const auto waist_it = serials.find(Microcontroller::Waist);
  if (waist_it == serials.end())
  {
    std::ostringstream o;
    o << "Didn't find Quori's Waist";
    throw std::runtime_error(o.str());
  }

  const auto base_it = serials.find(Microcontroller::Base);
  if (base_it == serials.end())
  {
    std::ostringstream o;
    o << "Didn't find Quori's Base";
    throw std::runtime_error(o.str());
  }

  std::cout << "Found microcontroller mappings:" << std::endl << serials;

  const static path RULES_PATH = "/etc/udev/rules.d/80-quori.rules";

  std::ofstream rules(RULES_PATH.string());
  if (!rules.is_open())
  {
    std::ostringstream o;
    o << "Unable to open \"" << RULES_PATH << "\" for writing. Make sure you run this program as superuser!";
    throw std::runtime_error(o.str());
  }

  for (const auto &mapping : serials)
  {
    rules << udev_rule(mapping.first, mapping.second);
  }

  // Make all ttyUSB* devices user-rw as well
  rules <<  "KERNEL==\"ttyUSB[0-9]*\", SUBSYSTEM==\"usb\", MODE=\"0666\"" << std::endl;

  rules.close();

  std::cout << "Successfully wrote udev rules!" << std::endl;

  bool should_reboot = true;
  for (;;)
  {
    std::cout << "You must reboot for changes to take effect. Would you like to reboot now? [Y/n]: ";
    char line[2];
    std::cin.getline(line, sizeof line);

    std::cout << line << std::endl;
    
    if (line[0] == 0) break;

    if (line[0] == 'Y' || line[0] == 'y')
    {
      should_reboot = true;
      break;
    }

    if (line[0] == 'N' || line[0] == 'n')
    {
      should_reboot = false;
      break;
    }
  }

  

  if (should_reboot)
  {
    std::cout << "Rebooting..." << std::endl;
    run("/sbin/reboot", {});
  }
  

  return EXIT_SUCCESS;
}
