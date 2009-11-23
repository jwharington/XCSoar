#include "TaskStats.hpp"
#include "Navigation/Aircraft.hpp"
#include <algorithm>
#include <assert.h>

#define N_AV 3

DistanceStat::DistanceStat():
  distance(0.0),
  speed(0.0),
  av_dist(N_AV),
  df(0.0),
  v_lpf(600.0/N_AV,false)
{

}

void 
DistanceRemainingStat::calc_speed(const ElementStat* es) 
{
  if (es->TimeRemaining>0) {
    speed = (distance/es->TimeRemaining);
  } else {
    speed = 0;
  }
}

void 
DistancePlannedStat::calc_speed(const ElementStat* es) 
{
  if (es->TimePlanned>0) {
    speed = (distance/es->TimePlanned);
  } else {
    speed = 0;
  }
}

void 
DistanceTravelledStat::calc_speed(const ElementStat* es) 
{
  if (es->TimeElapsed>0) {
    speed = (distance/es->TimeElapsed);
  } else {
    speed = 0;
  }
}

void 
DistanceStat::calc_incremental_speed(const double dt)
{  
  if ((dt>0) && (distance>0)) {
    if (av_dist.update(distance)) {
      for (unsigned i=0; i<(unsigned)(dt); i++) {
        double d_av = av_dist.average();
        double v = df.update(d_av)/(N_AV);
        double v_f = v_lpf.update(v);
        speed_incremental = -v_f;
        av_dist.reset();
      }
    }
  } else {    
    df.reset(distance,-speed*(N_AV));
    v_lpf.reset(-speed);
    speed_incremental = speed;
    av_dist.reset();
  }
}

void 
DistanceTravelledStat::calc_incremental_speed(const double dt)
{
  // negative of normal
  if ((dt>0) && (distance>0)) {
    for (unsigned i=0; i<(unsigned)(dt); i++) {
      if (av_dist.update(distance)) {
        double d_av = av_dist.average();
        double v = df.update(d_av)/(N_AV);
        double v_f = v_lpf.update(v);
        speed_incremental = v_f;
        av_dist.reset();
      }
    }
  } else {
    df.reset(distance,speed*(N_AV));
    v_lpf.reset(speed);
    speed_incremental = speed;
    av_dist.reset();
  }
}


void 
ElementStat::set_times(const double ts, 
                       const AIRCRAFT_STATE& state)
{
  TimeStarted = ts;
  TimeElapsed = std::max(state.Time-ts,0.0);
  TimeRemaining = solution_remaining.TimeElapsed;
  TimePlanned = TimeElapsed+TimeRemaining;
}

void
ElementStat::reset()
{
  initialised = false;

  calc_speeds(0);
}

void 
ElementStat::calc_speeds(const double dt)
{
  remaining_effective.calc_speed(this);
  remaining.calc_speed(this);
  planned.calc_speed(this);
  travelled.calc_speed(this);

  if (!initialised) {
    if ((dt>0) && (TimeElapsed>15)) {
      initialised=true;
    }
    remaining_effective.calc_incremental_speed(0.0);
    remaining.calc_incremental_speed(0.0);
    planned.calc_incremental_speed(0.0);
    travelled.calc_incremental_speed(0.0);
  } else {
    remaining_effective.calc_incremental_speed(dt);
    remaining.calc_incremental_speed(dt);
    planned.calc_incremental_speed(dt);
    travelled.calc_incremental_speed(dt);
  }
}


void
TaskStats::reset()
{
  total.reset();
  current_leg.reset();
}
