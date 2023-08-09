// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Math/WindowFilter.hpp"
#include "time/DeltaTime.hpp"
#include "NMEA/Derived.hpp"
#include "Settings.hpp"

struct MoreData;

class AverageVarioComputer {
  DeltaTime delta_time;

  WindowFilter<60> vario_avg_filter;
  WindowFilter<60> netto_avg_filter;

public:
  void Reset();

  void Compute(const MoreData &basic,
               bool circling, bool last_circling,
               DerivedInfo &calculated,
               const CirclingSettings &settings);
};
