#include "Csv.hpp"

using namespace quori_controller;

Csv::Row::Row(Csv &csv)
  : csv_(csv)
{
}

Csv::Row::~Row()
{
  csv_.appendRow(elements_);
}

Csv::Row &Csv::Row::operator<< (const std::string &value)
{
  elements_.push_back(value);
  return *this;
}

Csv::Row &Csv::Row::operator<< (const char *const value)
{
  elements_.push_back(value);
  return *this;
}

Csv::Csv(std::ostream &out)
  : out_(out)
{
}

Csv::~Csv()
{
  out_.flush();
  std::cerr << "Cleanup" << std::endl;
}

Csv::Ptr Csv::open(std::ostream &out)
{
  return Csv::Ptr(new Csv(out));
}

Csv::Row Csv::append()
{
  return Row(*this);
}

void Csv::appendRow(const std::vector<std::string> &elements)
{
  std::lock_guard<std::mutex> lock(mut_);
  
  for (auto it = elements.begin(); it != elements.end(); ++it)
  {
    if (it != elements.begin()) out_ << ",";
    out_ << *it;
  }
  out_ << std::endl;
}