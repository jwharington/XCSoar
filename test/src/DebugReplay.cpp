// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "DebugReplay.hpp"
#include "DebugReplayIGC.hpp"
#include "DebugReplayKML.hpp"
#include "DebugReplayNMEA.hpp"
#include "system/Args.hpp"
#include "system/PathName.hpp"
#include "Computer/Settings.hpp"
#include "util/StringCompare.hxx"

#include <string>
#include <string_view>

DebugReplay::DebugReplay()
  :glide_polar(1)
{
  Reset();
}

DebugReplay::~DebugReplay()
{
}

void DebugReplay::Reset()
{
  raw_basic.Reset();
  computed_basic.Reset();
  calculated.Reset();

  flying_computer.Reset();

  wrap_clock.Reset();

  qnh = AtmosphericPressure::Standard();
}

void
DebugReplay::Compute()
{
  computed_basic.Reset();
  (NMEAInfo &)computed_basic = raw_basic;
  wrap_clock.Normalise(computed_basic);

  FeaturesSettings features;
  features.nav_baro_altitude_enabled = true;
  computer.Fill(computed_basic, qnh, features);

  computer.Compute(computed_basic, last_basic, last_basic, calculated);
  flying_computer.Compute(glide_polar.GetVTakeoff(),
                          computed_basic, calculated,
                          calculated.flight);
}

static bool
HasSuffixBeforeSelector(const char *spec, const char *suffix)
{
  std::string_view view(spec);
  const std::size_t hash = view.find('#');
  if (hash != std::string_view::npos)
    view = view.substr(0, hash);

  const std::string base(view);
  return StringEndsWithIgnoreCase(base.c_str(), suffix);
}

DebugReplay *
CreateDebugReplay(Args &args)
{
  DebugReplay *replay;

  if (!args.IsEmpty() && HasSuffixBeforeSelector(args.PeekNext(), ".igc")) {
    replay = DebugReplayIGC::Create(args.ExpectNextPath());
  } else if (!args.IsEmpty() &&
             (HasSuffixBeforeSelector(args.PeekNext(), ".kml") ||
              HasSuffixBeforeSelector(args.PeekNext(), ".kmz"))) {
    replay = DebugReplayKML::Create(args.ExpectNextPath());
  } else {
    const auto driver_name = args.ExpectNextT();
    const auto input_file = args.ExpectNextPath();
    replay = DebugReplayNMEA::Create(input_file, driver_name);
  }

  return replay;
}
