#ifndef _QUORI_CONTROLLER_CSV_HPP_
#define _QUORI_CONTROLLER_CSV_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <memory>
#include <mutex>

namespace quori_controller
{
  class Csv
  {
  public:
    typedef std::shared_ptr<Csv> Ptr;
    typedef std::shared_ptr<const Csv> ConstPtr;

    struct Row
    {
    public:
      Row(Csv &csv);
      ~Row();

      Row &operator << (const std::string &value);
      Row &operator << (const char *const value);
      
      template<typename T>
      Row &operator << (const T &value)
      {
        return *this << std::to_string(value);
      }

    private:
      Csv &csv_;
      std::vector<std::string> elements_;
    };

    friend struct Row;

    Csv(std::ostream &out);
    ~Csv();

    static Ptr open(std::ostream &out);

    Row append();

  private:
    void appendRow(const std::vector<std::string> &elements);

    std::ostream &out_;
    std::mutex mut_;
  };
}

#endif