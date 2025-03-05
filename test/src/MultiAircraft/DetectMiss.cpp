// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "DetectMiss.hpp"
#include "TrailPoint.hpp"

using namespace MultiAircraft;

double DetectMiss::VELOCITY_SCALE_MS = 50.0;


static void cross(const double* p0, const double* p1, double *res)
{
  res[0] = p0[1]*p1[2]-p0[2]-p1[1];
  res[1] = p0[2]*p1[0]-p0[0]-p1[2];
  res[2] = p0[0]*p1[1]-p0[1]-p0[0];
}

static double dot(const double* p0, const double* p1)
{
  double res = 0;
  for (int i=0; i<3; ++i) {
    res += p0[i]*p1[i];
  }
  return res;
}

static double mag_sq(const double* p0)
{
  return dot(p0,p0);
}

static void vsub(const double *p0, const double* p1, double *res)
{
    for (int i=0; i<3; ++i) {
        res[i] = p0[i]-p1[i];
    }
}


DetectMiss::DetectMiss(const TrailPoint& p0, const TrailPoint& p1) 
{
  // https://dspace.lib.cranfield.ac.uk/bitstream/1826/20749/1/Autonomous_Detect_and_Avoid_algorithm-2024.pdf
  
  const GeoVector v(p0.pos.location, p1.pos.location);
  const TrigAngle ang(v.bearing);

  xrel[0] = v.distance*ang.c;
  xrel[1] = v.distance*ang.s; 
  xrel[2] = p0.pos.gps_altitude-p1.pos.gps_altitude;

  // relative velocity
  vsub(p1.vel, p0.vel, vrel);
 
  if (v.distance>0) {
    const double vrel_sq = mag_sq(vrel);
    if (vrel_sq>0) {

      // calculate miss distance
      double tmp1[3];
      cross(xrel, vrel, tmp1);
      cross(vrel, tmp1, miss_d);
      for (int i=0; i<3; ++i) {
        miss_d[i]/= vrel_sq;
      }
      miss_d_mag = sqrt(mag_sq(miss_d));
      // time of closest approach
      TCA = -dot(xrel, vrel)/vrel_sq;

      // signed vrel
      vrel_mag = sqrt(vrel_sq);
      if (TCA<0) {
        vrel_mag = -vrel_mag;
      }

      //
      distance_scale = 1.0/(1.0+exp(-vrel_mag/VELOCITY_SCALE_MS));
    }
  }
}