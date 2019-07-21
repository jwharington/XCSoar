/*
  Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2019 The XCSoar Project
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

#include "IGCFRInfo.hpp"
#include "util/StringAPI.hxx"

void IGCFRInfo::CheckCorrection()
{
  geoid_correction = 0;
  if (nullptr != StringFind(fr_type, "FLARM") || nullptr != StringFind(fr_type, "Flarm")) {
    if (nullptr != StringFind(fw_version, ":6.") ||
        nullptr != StringFind(fw_version, "06.") ||
        nullptr != StringFind(fw_version, ",6.") ||
        nullptr != StringFind(fw_version, "05.") ||
        nullptr != StringFind(fw_version, "04.") ||

        // powerflarms
        nullptr != StringFind(fw_version, "3.40")
        ) {
      geoid_correction = -1;
    }
  } else if (nullptr != StringFind(fr_type, "ClearNav II")) {
    if (nullptr != StringFind(fw_version, "1.3") || nullptr != StringFind(fw_version, "1.4")) {
      geoid_correction = -1;
    }
  } else if (nullptr != StringFind(fr_type, "FILSER,LX20")) {
    if (nullptr != StringFind(fw_version, "5.0") || nullptr != StringFind(fw_version, "5.1")) {
      geoid_correction = -1;
    }
    // 5.2 is good
  } else if (nullptr != StringFind(fr_type, "FILSER,LX5000IGC")) {
    if (nullptr != StringFind(fw_version, "6.1")) {
      geoid_correction = -1;
    }
  } else if (nullptr != StringFind(fr_type, "CNv-IGC")) {
    if (nullptr != StringFind(fw_version, "ADC 3.6") ||
        nullptr != StringFind(fw_version, "ADC 2.7") ||
        nullptr != StringFind(fw_version, "ADC 2.6")
        ) {
      geoid_correction = -1;
    }
  } else if (nullptr != StringFind(fr_type, "Nielsen Kellerman, ClearNav-IGC")) {
    if (nullptr != StringFind(fw_version, "40 2011-6-17")) {
      geoid_correction = 0;
    } else if (nullptr != StringFind(fw_version, "38 2010-7-7")) {
      geoid_correction = 0;
    } else if (nullptr != StringFind(fw_version, "34 2009-9-30")) {
      geoid_correction = 0;
    } else {
      geoid_correction = -1;
    }
  } else if (nullptr != StringFind(fr_type, "Triadis")) {
    geoid_correction = -1;
  } else if (nullptr != StringFind(fr_type, "Oudie-IGC")) {
    if (nullptr != StringFind(fw_version, ":9.")) {
      geoid_correction = 1;
    }
  } else if (nullptr != StringFind(fr_type, "LXNAVIGATION,LX_Colibri_II")) {
    if (nullptr != StringFind(fw_version, ":1.9")) {
      geoid_correction = 0; // ??? HW2.3
    }
  } else if (nullptr != StringFind(fr_type, "LX Eos")) {
    if (nullptr != StringFind(fw_version, ":1.6") || // ? check
        nullptr != StringFind(fw_version, ":1.7") ||
        nullptr != StringFind(fw_version, ":1.8")
        ) {
      geoid_correction = 0;
    }
  } else if (nullptr != StringFind(fr_type, "LXNAV,NANO4")) {
    geoid_correction = 0;
  } else if (nullptr != StringFind(fr_type, "LXNAV,NANO3")) {
    geoid_correction = 0;
  } else if (nullptr != StringFind(fr_type, "LXNAV,NANO2")) {
    geoid_correction = 0;
  } else if (nullptr != StringFind(fr_type, "LXNAV,NANO")) {
    geoid_correction = 0;
  } else if (nullptr != StringFind(fr_type, "LXNAV,LX90")) {
    if (nullptr != StringFind(fw_version, ":7.13") ||
        nullptr != StringFind(fw_version, "with WGS84 Ellipsoid GPS altitude datum")) {
      geoid_correction = -1;
    }
  } else if (nullptr != StringFind(fr_type, "EW-Avionics-microRecorder")) {
    if (nullptr != StringFind(fw_version, ":8.0")) {
    } else {
      geoid_correction = 1;
    }
  }
}

void IGCFRInfo::clear()
{
  geoid_correction = 0;
  fr_type[0] = 0;
  fw_version[0] = 0;
}
