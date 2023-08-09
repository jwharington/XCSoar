// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "TurnModeList.hpp"
#include "TrailPoint.hpp"

namespace MultiAircraft {

class TrailPointList: public std::list<TrailPoint>
{
 public:
  TurnModeList gen_turnmodelist(const TimeStamp t0, const TimeStamp t1) const
  {
    bool started = false;
    TurnModeList turn_mode_list;
    for (auto&& p: *this) {
      bool do_append = false;
      if (p.within_time(t0, t1)) {
        do_append = !started;
        if (started && (turn_mode_list.back() != p.turn_mode)) {
          do_append = true;
        }
        started = true;
        if (do_append)
          turn_mode_list.push_back(p.turn_mode);
      }
    }
    return turn_mode_list;
  }
};


}
