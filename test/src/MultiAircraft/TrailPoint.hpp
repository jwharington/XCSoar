// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Replay/CatmullRomInterpolator.hpp"
#include "NMEA/CirclingInfo.hpp"
#include "Visibility.hpp"
#include "DetectMiss.hpp"
#include "Geo/Math.hpp"
#include "Geo/SpeedVector.hpp"
#include "time/Stamp.hpp"
#include <unordered_map>

namespace MultiAircraft {

typedef std::pair<Aspect, DetectMiss> AuxiliaryPair;

struct TrailPoint {
  TrailPoint(const CatmullRomInterpolator::Record& _pos,
             const GeoVector& _trk,
             const CirclingMode& _turn_mode,
             const double _fix_acc,
             const bool _actual) noexcept: pos(_pos), trk(_trk), turn_mode(_turn_mode), fix_acc(_fix_acc), actual(_actual) {};

  bool within_time(const TimeStamp t0, const TimeStamp t1) const {
    return (pos.time >= t0) && (pos.time <= t1);
  }
  const CatmullRomInterpolator::Record pos;
  const GeoVector trk;
  const CirclingMode turn_mode;
  const double fix_acc;
  const bool actual;

  void update_reconstruction(const TrailPoint& prev, const SpeedVector& wind);
  SpeedVector v_wind = SpeedVector(0,0);
  double v_ias = 0;
  double v_tas = 0;
  double roc = 0.0;
  Angle bank_angle = Angle::Native(0);
  Angle turn_rate_wind = Angle::Native(0);
  Angle pitch_angle = Angle::Native(0);
  Angle yaw_angle = Angle::Native(0);
  double nv = 0;
  double load_factor = 0;
  double nturn = 0;
  bool plausible = true;
  double vel[3];
  typedef std::unordered_map<unsigned, AuxiliaryPair> AuxiliaryList;
  const AuxiliaryPair& lookup_auxiliary(const unsigned id_target) const;
  bool present(const unsigned id_target) const;
  void add_auxiliary(const unsigned id_target, const AuxiliaryPair &p);

 private:
  TrailPoint(const TrailPoint&) = delete;
  AuxiliaryList auxiliaries;

  static constexpr double G = 9.81;
  // maximum plausible forward acceleration in g (allowing for some numerical jitter)
  static constexpr double ACCEL_MAX_PLAUSIBLE_G = 1.2;
  // maximum plausible turn acceleration in g (allowing for some numerical jitter)
  static constexpr double NTURN_MAX_PLAUSIBLE_G = 10.0;

};

}
