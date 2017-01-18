#include "Visibility.hpp"
#include <stdio.h>

using namespace MultiAircraft;

/*
static double haversine_distance(const double elevation_1, const double azimuth_1,
                                 const double elevation_2, const double azimuth_2)
{
  const double daz = fabs(azimuth_1-azimuth_2);
  return acos(sin(elevation_1)*sin(elevation_2)+cos(elevation_1)*cos(elevation_2)*cos(daz));
}
*/

static Angle boresight_angle(const Aspect& aspect)
{
  return Angle::acos(aspect.elevation_angle.cos()*aspect.azimuth_angle.Absolute().cos());
}

Visibility::Visibility(const Aspect& aspect) {

  angular_size = Angle::FromXY(aspect.range, 10/2)*2;

  if (aspect.azimuth_angle.Absolute() > Angle::Degrees(140)+angular_size/2) {
    occlusion = 1;
  } else if (aspect.elevation_angle < -Angle::Degrees(20) -angular_size/2) {
    occlusion = 1;
  } else {
    occlusion = 0;
  }
  if (aspect.inclination_angle.Absolute() < Angle::Degrees(20)+angular_size/2) {
    // focus along horizon
    focus_factor = 1;
  } else if (boresight_angle(aspect) < Angle::Degrees(30)+angular_size/2) {
    // focus forward
    focus_factor = 1;
  } else {
    focus_factor = 0.5;
  }
}

//////////////////

struct Point {
  double x[3];
  Point interp(const Point& last, const double t) {
    Point p;
    for (int i=0; i<3; ++i) {
      p.x[i] = x[i]*t+last.x[i]*(1-t);
    }
    return p;
  }
};

void test_visibility()
{
  EulerAngles ea;
  const double z = -2;
  const double x = 3;
  Point points[] = {
    {22+x,0,3+z},
    {19+x,4,6+z},
    {-5+x,6,3+z},
    {-7+x,4,0},
    {-9+x,0,-2+z}
  };
  Point p_last;
  bool first = true;
  for (auto&& p : points) {
    if (first) {
      p_last = p;
      first = false;
      continue;
    }
    for (double t=0; t<= 1; t+= 0.05) {
      const Point n = p.interp(p_last, t);
      const Aspect a= ea.get_aspect(n.x);
      printf("%+.1f %+.1f %+.1f\t%+.1f %+.1f\n", n.x[0], n.x[1], n.x[2], a.elevation_angle.Degrees(), a.azimuth_angle.Degrees());
    }
    p_last = p;
  }
}
