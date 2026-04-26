// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include <list>

#include "TrajectoryTokenizerQTC3D.hpp"
#include "TrailPointList.hpp"

#include "Geo/GeoVector.hpp"

#include <cmath>

namespace MultiAircraft
{

  double
  TrajectoryTokenizerQTC3D::WrapAngleRad(const double x) noexcept
  {
    double y = std::fmod(x, 2.0 * M_PI);
    if (y > M_PI)
      y -= 2.0 * M_PI;
    if (y < -M_PI)
      y += 2.0 * M_PI;
    return y;
  }

  char
  TrajectoryTokenizerQTC3D::SignToken(const double value, const double eps) noexcept
  {
    if (value > eps)
      return '+';
    if (value < -eps)
      return '-';
    return '0';
  }

  std::vector<std::string>
  TrajectoryTokenizerQTC3D::TokenizePair(const TrailPointList &a,
                                         const TrailPointList &b) const
  {
    std::vector<std::string> tokens;
    if (a.size() < 2 || b.size() < 2)
      return tokens;

    auto ia = a.begin();
    auto ib = b.begin();

    bool have_prev = false;
    const TrailPoint *prev_a = nullptr;
    const TrailPoint *prev_b = nullptr;

    while (ia != a.end() && ib != b.end())
    {
      const double dt_time = ((*ia).pos.time - (*ib).pos.time).count();
      if (std::fabs(dt_time) > config.max_time_delta_seconds)
      {
        if (dt_time < 0)
          ++ia;
        else
          ++ib;
        continue;
      }

      const auto &curr_a = *ia;
      const auto &curr_b = *ib;

      if (have_prev && prev_a != nullptr && prev_b != nullptr)
      {
        const double dt = (curr_a.pos.time - prev_a->pos.time).count();
        if (dt >= config.min_dt_seconds)
        {
          const bool plausible = prev_a->plausible && prev_b->plausible &&
                                 curr_a.plausible && curr_b.plausible;

          if (config.include_implausible || plausible)
          {
            const GeoVector prev_sep(prev_a->pos.location, prev_b->pos.location);
            const GeoVector curr_sep(curr_a.pos.location, curr_b.pos.location);

            const double prev_alt_a = config.use_baro_altitude ? prev_a->pos.baro_altitude : prev_a->pos.gps_altitude;
            const double prev_alt_b = config.use_baro_altitude ? prev_b->pos.baro_altitude : prev_b->pos.gps_altitude;
            const double curr_alt_a = config.use_baro_altitude ? curr_a.pos.baro_altitude : curr_a.pos.gps_altitude;
            const double curr_alt_b = config.use_baro_altitude ? curr_b.pos.baro_altitude : curr_b.pos.gps_altitude;

            const char radial = SignToken(curr_sep.distance - prev_sep.distance, config.radial_zero_m);
            const char vertical = SignToken((curr_alt_b - curr_alt_a) - (prev_alt_b - prev_alt_a), config.vertical_zero_m);

            const double rel_prev = WrapAngleRad(prev_sep.bearing.Radians() - prev_a->trk.bearing.Radians());
            const double rel_curr = WrapAngleRad(curr_sep.bearing.Radians() - curr_a.trk.bearing.Radians());
            const char lateral = SignToken(WrapAngleRad(rel_curr - rel_prev), config.lateral_zero_rad);

            std::string token = std::string("QTC3D(r=") + radial +
                                ",z=" + vertical +
                                ",l=" + lateral + ")";

            if (config.include_bank_state)
            {
              const char bank_a = SignToken(curr_a.bank_angle.Degrees(), config.bank_zero_deg);
              const char bank_b = SignToken(curr_b.bank_angle.Degrees(), config.bank_zero_deg);
              token += std::string("|BANK(bA=") + bank_a + ",bB=" + bank_b + ")";
            }

            if (config.include_bank_trend)
            {
              const double d_bank_a = curr_a.bank_angle.Degrees() - prev_a->bank_angle.Degrees();
              const double d_bank_b = curr_b.bank_angle.Degrees() - prev_b->bank_angle.Degrees();
              const char dbank_a = SignToken(d_bank_a, config.bank_delta_zero_deg);
              const char dbank_b = SignToken(d_bank_b, config.bank_delta_zero_deg);
              token += std::string("|DBANK(dbA=") + dbank_a + ",dbB=" + dbank_b + ")";
            }

            tokens.push_back(std::move(token));
          }
        }
      }

      prev_a = &curr_a;
      prev_b = &curr_b;
      have_prev = true;
      ++ia;
      ++ib;
    }

    return tokens;
  }

} // namespace MultiAircraft
