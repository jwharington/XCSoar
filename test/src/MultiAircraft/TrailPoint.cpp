// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "TrailPoint.hpp"
#include "Math/Vector.hpp"
#include "Atmosphere/AirDensity.hpp"

using namespace MultiAircraft;

static const double LIFT_CURVE_SLOPE = 2*M_PI;
static const double WING_LOADING = 50; // kg/m^2

static SpeedVector vector_wind(const GeoVector& p, const SpeedVector& wind)
{
  const Vector ve = Vector(SpeedVector(p.bearing, p.distance)) + Vector(wind);
  return SpeedVector(ve.y, ve.x);
}

static Angle turnrate(const SpeedVector &v_wind, const SpeedVector &v_wind_last)
{
  if (!v_wind.norm || !v_wind_last.norm) {
    return Angle::Native(0);
  }
  return (v_wind.bearing-v_wind_last.bearing).AsDelta();
}

void TrailPoint::update_reconstruction(const TrailPoint& prev, const SpeedVector& wind)
{
  v_wind = vector_wind(trk, wind);
  v_ias = v_wind.norm/AirDensityRatio(pos.gps_altitude);
  roc = (pos.gps_altitude - prev.pos.gps_altitude);

  {
    const Vector vv = Vector(v_wind);
    vel[0] = vv.x;
    vel[1] = vv.y;
    vel[2] = -roc; // z down
  }

  turn_rate_wind = turnrate(v_wind, prev.v_wind).Half() + prev.turn_rate_wind.Half();
  bank_angle = Angle::Radians(atan(turn_rate_wind.Radians() * v_wind.norm / G));
  if (v_ias>10.0) {
    yaw_angle = (v_wind.bearing + Angle::Radians(WING_LOADING*9.81*bank_angle.tan()/(0.5*1.225*v_ias*v_ias*LIFT_CURVE_SLOPE))).AsDelta();
  } else {
    yaw_angle = (v_wind.bearing).AsDelta();
  }

  nv = (v_wind.norm - prev.v_wind.norm)/G;
  if (fabs(nv)> ACCEL_MAX_PLAUSIBLE_G) {
    plausible = false;
  }
  if (fabs(nv)<1.0) {
    pitch_angle = Angle::asin(-nv).Fraction(prev.pitch_angle, 0.5);
  } else {
    pitch_angle = Angle::Native(0);
  }

  nturn = fabs(v_wind.norm*turn_rate_wind.Radians()/G);
  if (nturn > NTURN_MAX_PLAUSIBLE_G) {
    plausible = false;
  }

}


bool TrailPoint::present(const unsigned id_target) const
{
  return (auxiliaries.find(id_target) != auxiliaries.end());
}

const AuxiliaryPair& TrailPoint::lookup_auxiliary(const unsigned id_target) const
{
  auto i = auxiliaries.find(id_target);
  if (i != auxiliaries.end()) {
    return i->second;
  }
  static const AuxiliaryPair def = AuxiliaryPair(Aspect(), DetectMiss());
  return def;
}

void TrailPoint::add_auxiliary(const unsigned id_target, const AuxiliaryPair &p)
{
  auxiliaries[id_target] = p;
}
