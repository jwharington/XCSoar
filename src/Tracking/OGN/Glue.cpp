/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
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

#include "Glue.hpp"
#include "Settings.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/Derived.hpp"
#include "net/State.hpp"

#include <assert.h>

OGN::Glue::Glue(boost::asio::io_context &io_context,
                Handler *_handler)
  :client(io_context, _handler)
{
}

OGN::Glue::~Glue()
{
}

inline bool
OGN::Glue::IsConnected() const
{
  switch (GetNetState()) {
  case NetState::UNKNOWN:
    /* we don't know if we have an internet connection - be
       optimistic, and assume everything's ok */
    return true;

  case NetState::DISCONNECTED:
    return false;

  case NetState::CONNECTED:
    return true;

  case NetState::ROAMING:
    return roaming;
  }

  assert(false);
  gcc_unreachable();
}


void
OGN::Glue::Tick(const NMEAInfo &basic,
                const DerivedInfo &calculated)
{
  if (basic.location_available && !basic.gps.real)
    /* disable in simulator/replay */
    return;

  if (enabled && clock.CheckAdvance(basic.clock, interval))
    client.SendTrafficRequest(basic.location);
}


void
OGN::Glue::SetSettings(const Settings &settings)
{
  enabled = settings.enabled;
  roaming = settings.roaming;
  interval = std::chrono::seconds(settings.interval);
  client.SetSettings(settings);
}
