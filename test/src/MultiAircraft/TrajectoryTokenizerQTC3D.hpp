// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include <string>
#include <vector>

namespace MultiAircraft
{

  class TrailPointList;

  class TrajectoryTokenizerQTC3D
  {
  public:
    struct Config
    {
      double max_time_delta_seconds = 1.0;
      double min_dt_seconds = 0.4;
      bool use_baro_altitude = true;
      bool include_implausible = false;
      double radial_zero_m = 0.5;
      double vertical_zero_m = 0.2;
      double lateral_zero_rad = 1e-3;
      bool include_bank_state = true;
      double bank_zero_deg = 5.0;
      bool include_bank_trend = true;
      double bank_delta_zero_deg = 1.0;
    };

    TrajectoryTokenizerQTC3D() noexcept = default;

    explicit TrajectoryTokenizerQTC3D(const Config &_config) noexcept
      : config(_config) {}

    std::vector<std::string> TokenizePair(const TrailPointList &a,
                                          const TrailPointList &b) const;

  private:
    Config config;

    static double WrapAngleRad(double x) noexcept;
    static char SignToken(double value, double eps = 1e-3) noexcept;
  };

} // namespace MultiAircraft
