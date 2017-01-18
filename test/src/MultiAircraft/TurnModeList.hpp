#pragma once
#include "NMEA/CirclingInfo.hpp"
#include <string>

namespace MultiAircraft {

class TurnModeList: public std::list<CirclingMode>
{
 public:
  std::string string() const {
    std::string out;
    bool first = true;
    for (auto m = begin(); m != end(); ++m) {
      if (!first) {
        out += ", ";
      }
      out += to_string(*m);
      first = false;
    }
    return out;
  }
  static std::string to_string(const CirclingMode mode) {
    return symbols[(int)mode];
  }
 private:
  static const constexpr char* const symbols[] = {"cruise",
                                                  "entry",
                                                  "circling",
                                                  "exit"};

};

}
