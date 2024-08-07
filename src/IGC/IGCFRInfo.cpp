// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "IGCFRInfo.hpp"
#include "util/StringAPI.hxx"

#include <list>
#include <regex>
#include <cstdio>
#include <cstring>
#include "io/FileLineReader.hpp"
#include "system/Path.hpp"

struct IGCFRInfoDBType {
  int geoid_correction;
  char fr_type[80];
  char fw_version[80];

  bool check(IGCFRInfo& other) const {
    if ((nullptr != StringFind(other.fr_type, fr_type))
        && (nullptr != StringFind(other.fw_version, fw_version))) {
      other.geoid_correction = geoid_correction;
      //      printf("match %d %s %s\n", geoid_correction, fr_type, fw_version);
      return true;
    }
    return false;
  }
};

typedef std::list<IGCFRInfoDBType> IGCFRInfoDB;

static IGCFRInfoDB fw_info_db;
static bool fw_info_init = false;

static bool IGCFRInfoDB_check(IGCFRInfo& info)
{
  for (auto &d: fw_info_db) {
    if (d.check(info))
      return true;
  }
  return false;
}

bool IGCFRInfoDB_init(const char* file_path)
{
  fw_info_init = true;
  try {
    Path path = Path(file_path);
    FileLineReaderA reader(path);
    char *line;
    do {
      line = reader.ReadLine();
      if (nullptr != line) {
        IGCFRInfoDBType d;
        std::string sline(line);
        std::regex pattern("(-?[012]) '(.*)' '(.*)'");
        std::smatch pieces_match;
        if (std::regex_match(sline, pieces_match, pattern)) {
          if (4 == pieces_match.size()) {
            d.geoid_correction = atoi(pieces_match[1].str().c_str());
            strncpy(d.fr_type,pieces_match[2].str().c_str(),80);
            strncpy(d.fw_version,pieces_match[3].str().c_str(),80);
            fw_info_db.push_back(d);
          }
        }
      }
    } while (nullptr != line);
    return true;
  } catch (...) {
    return false;
  }
}

void IGCFRInfo::CheckCorrection()
{
  if (!fw_info_init) {
    IGCFRInfoDB_init("igc_fr_geoid.txt");
  }

  geoid_correction = 0;

  if (IGCFRInfoDB_check(*this)) {
    return;
  }

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
  strncpy(fr_type,":Unknown",9);
  fw_version[0] = 0;
}
