#include "Vignette.hpp"
#include "TrailPoint.hpp"

using namespace MultiAircraft;

const GeoPoint Vignette::calc_traildrift() const
{
  const GeoPoint tp1 = FindLatitudeLongitude(origin, wind.bearing, wind.norm);
  return tp1 - origin;
}

void Vignette::finalise()
{
  traildrift = calc_traildrift();
  proj = FlatProjection(origin);
  scale = proj.GetApproximateScale();
}

const FlatPoint Vignette::project_loc_wind(const TrailPoint& p) const
{
  const GeoPoint loc_drift = p.pos.location.Parametric(traildrift, p.pos.time-time_start);
  return proj.ProjectFloat(loc_drift)*scale;
}
