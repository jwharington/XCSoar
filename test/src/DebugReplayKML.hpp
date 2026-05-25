// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "DebugReplay.hpp"
#include "Geo/GeoPoint.hpp"

#include <cstddef>
#include <vector>

class Path;

class DebugReplayKML final : public DebugReplay {
public:
  struct Fix {
    BrokenDateTime date_time_utc;
    GeoPoint location;
    double gps_altitude = 0;
  };

private:
  std::vector<Fix> fixes;
  std::size_t index = 0;
  double h_acc = 25.0;

  explicit DebugReplayKML(std::vector<Fix> &&_fixes) noexcept;

public:
  bool Next() override;
  bool Rewind() override;

  static DebugReplay *Create(Path input_file);
  static std::vector<std::string> ListPointTimelineSources(Path input_file);

  std::string GetTypeInfo() const override;
  std::string GetIdentifier() const override;
  double GetHAccuracy() const override;

private:
  void CopyFromFix(const Fix &fix);
};
