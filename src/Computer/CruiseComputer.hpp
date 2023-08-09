// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Geo/Flat/FlatProjection.hpp"
#include "Geo/SearchPointVector.hpp"
#include "time/Stamp.hpp"

struct MoreData;
struct DerivedInfo;

class CruiseComputer {
public:
  void Reset(DerivedInfo &calculated);
  void ResetStats(const MoreData &basic, DerivedInfo &calculated);

  void Compute(const MoreData &basic,
               DerivedInfo &calculated);

 private:
  SearchPointVector search_hull;
  /** Task projection used for flat-earth representation */
  FlatProjection projection;
  TimeStamp cruise_start_time;
};

