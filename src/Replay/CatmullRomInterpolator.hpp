/*
  Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#ifndef XCSOAR_CATMULL_ROM_INTERPOLATOR_HPP
#define XCSOAR_CATMULL_ROM_INTERPOLATOR_HPP

#include "Math/Util.hpp"
#include "Geo/Math.hpp"
#include "Geo/GeoPoint.hpp"
#include "Geo/GeoVector.hpp"
#include "util/Clamp.hpp"
#include "Geo/SpeedVector.hpp"
#include "Geo/Flat/FlatProjection.hpp"
#include "Geo/Flat/FlatPoint.hpp"

#include <algorithm>
#include <cassert>

/**
 * A Catmull-Rom splines interpolator
 * @see http://www.cs.cmu.edu/~462/projects/assn2/assn2/catmullRom.pdf
 * @see http://algorithmist.net/docs/catmullrom.pdf
 */
class CatmullRomInterpolator
{
public:
  struct Record {
    GeoPoint location;
    double gps_altitude;
    double baro_altitude;
    double time;
  };

private:
  const double alpha;

  unsigned num;
  Record p[4];

public:
  CatmullRomInterpolator(double _alpha):alpha(_alpha)
  {
    Reset();
  }

  void
  Reset()
  {
    num = 0;
  }

  void
  Update(double t, GeoPoint location, double alt, double palt)
  {
    if (num && (t <= p[3].time))
      return;

    if (num < 4)
      num++;

    std::copy(p + 1, p + 4, p);

    p[3].location = location;
    p[3].gps_altitude = alt;
    p[3].baro_altitude = palt;
    p[3].time = t;
  }

  bool
  Ready() const
  {
    return (num == 4);
  }

  GeoVector
  GetVector(double _time, const SpeedVector wind) const
  {
    assert(Ready());

    if ((p[2].time - p[1].time) <= 0)
      return GeoVector(0, Angle::Zero());

    const auto u = GetTimeFraction(_time);
    const auto speed2 = p[1].location.DistanceS(p[2].location) / (p[2].time - p[1].time);
    const auto speed1 = p[0].location.DistanceS(p[1].location) / (p[1].time - p[0].time);

    const Record r0 = Interpolate(_time - 0.25, wind);
    const Record r1 = Interpolate(_time + 0.25, wind);
    const Angle bearing = r0.location.Bearing(r1.location);

    return GeoVector(speed2*u+speed1*(1-u), bearing);
  }

  gcc_pure
  bool
  IsActual(double _time) const
  {
    return (fabs(_time-p[1].time)<0.01) || (fabs(_time-p[2].time)<0.01);
  }

  gcc_pure
  Record
  Interpolate(double _time, const SpeedVector wind) const
  {
    assert(Ready());

    const auto u = GetTimeFraction(_time);

    /*
      ps = ( c0   c1    c2  c3)
      [  0    1     0   0] 1
      [ -t    0     t   0] u
      [ 2t  t-3  3-2t  -t] u^2
      [ -t  2-t   t-2   t] u^3
    */

    const auto u2 = Square(u);
    const auto u3 = u2 * u;
    const double c[4]= {-alpha * u3 + 2 * alpha * u2 - alpha * u,
                        (2 - alpha) * u3 + (alpha - 3) * u2 + 1,
                        (alpha - 2) * u3 + (3 - 2 * alpha) * u2 + alpha * u,
                        alpha * u3 - alpha * u2};

    /*
    const GeoPoint dloc = FindLatitudeLongitude(p[0].location, wind.bearing, wind.norm);
    const FlatProjection projection(p[0].location);
    const GeoPoint wind_drift = p[0].location - dloc;
    const FlatPoint fp[4] = {
      projection.ProjectFloat(p[0].location),
      projection.ProjectFloat(p[1].location + wind_drift * (p[1].time-p[0].time)),
      projection.ProjectFloat(p[2].location + wind_drift * (p[2].time-p[0].time)),
      projection.ProjectFloat(p[3].location + wind_drift * (p[3].time-p[0].time))
    };

    FlatPoint fr;
    fr.x = fp[0].x * c[0] + fp[1].x * c[1] + fp[2].x * c[2] + fp[3].x * c[3];
    fr.y = fp[0].y * c[0] + fp[1].y * c[1] + fp[2].y * c[2] + fp[3].y * c[3];

    Record r;

    r.location = projection.Unproject(fr) - wind_drift * (u*(p[2].time-p[1].time)+p[1].time-p[0].time);
    */
    Record r;
    r.location.latitude =
        p[0].location.latitude*c[0] + p[1].location.latitude*c[1] +
        p[2].location.latitude*c[2] + p[3].location.latitude*c[3];
    r.location.longitude =
        p[0].location.longitude*c[0] + p[1].location.longitude*c[1] +
        p[2].location.longitude*c[2] + p[3].location.longitude*c[3];

    r.gps_altitude =
        p[0].gps_altitude * c[0] + p[1].gps_altitude * c[1] +
        p[2].gps_altitude * c[2] + p[3].gps_altitude * c[3];

    r.baro_altitude =
        p[0].baro_altitude * c[0] + p[1].baro_altitude * c[1] +
        p[2].baro_altitude * c[2] + p[3].baro_altitude * c[3];

    r.time = _time;

    return r;
  }

  double
  GetMinTime() const
  {
    assert(Ready());

    return p[0].time;
  }

  double
  GetMaxTime() const
  {
    assert(Ready());

    return std::max({0., p[0].time, p[1].time, p[2].time, p[3].time});
  }

  bool
  NeedData(const double t_simulation) const
  {
    return !Ready() || (p[2].time <= t_simulation);
  }

private:
  double
  GetTimeFraction(const double time) const
  {
    assert(Ready());
    assert(p[2].time > p[1].time);

    const auto fraction = (time - p[1].time) / (p[2].time - p[1].time);

    return Clamp(fraction, 0., 1.);
  }
};

#endif
