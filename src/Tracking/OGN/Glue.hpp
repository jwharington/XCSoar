/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2020 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#ifndef XCSOAR_TRACKING_OGN_GLUE_HPP
#define XCSOAR_TRACKING_OGN_GLUE_HPP

#include "Client.hpp"
#include "time/GPSClock.hpp"

struct NMEAInfo;
struct DerivedInfo;

namespace OGN {

struct Settings;
class Queue;

class Glue {
  Client client;
  std::chrono::steady_clock::duration interval{};
  GPSClock clock;

  bool enabled = false;
  bool roaming = true;

public:
  Glue(boost::asio::io_context &io_context, Handler *_handler);
  ~Glue();

  void SetSettings(const Settings &settings);

  void Tick(const NMEAInfo &basic, const DerivedInfo &calculated);

private:
  gcc_pure
  bool IsConnected() const;

};

} /* namespace OGN */

#endif
