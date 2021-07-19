#pragma once

#include "Replay/CatmullRomInterpolator.hpp"
#include "NMEA/CirclingInfo.hpp"
#include "Visibility.hpp"
#include <unordered_map>

namespace MultiAircraft {

struct TrailPoint {
  TrailPoint(const CatmullRomInterpolator::Record& _pos,
             const GeoVector& _trk,
             const CirclingMode& _turn_mode,
             const double _fix_acc,
             const bool _actual) noexcept: pos(_pos), trk(_trk), turn_mode(_turn_mode), fix_acc(_fix_acc), actual(_actual) {};

  bool within_time(const double t0, const double t1) const {
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
  Angle bank_angle = Angle::Native(0);
  Angle turn_rate_wind = Angle::Native(0);
  Angle pitch_angle = Angle::Native(0);
  Angle yaw_angle = Angle::Native(0);
  double nv = 0;
  double nturn = 0;
  bool plausible = true;
  typedef std::unordered_map<unsigned, Aspect> AspectList;
  const Aspect lookup_aspect(const unsigned id_target) const;
  bool present(const unsigned id_target) const;

  AspectList aspects;

 private:
  TrailPoint(const TrailPoint&) = delete;

  static constexpr double G = 9.81;
  // maximum plausible forward acceleration in g (allowing for some numerical jitter)
  static constexpr double ACCEL_MAX_PLAUSIBLE_G = 1.2;
  // maximum plausible turn acceleration in g (allowing for some numerical jitter)
  static constexpr double NTURN_MAX_PLAUSIBLE_G = 10.0;

};

}
