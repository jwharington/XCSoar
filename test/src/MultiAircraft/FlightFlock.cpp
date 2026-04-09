// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "FlightFlock.hpp"
#include "AircraftModel.hpp"
#include "Geo/Flat/FlatPoint.hpp"
#include <iomanip> // std::setprecision
#include <fstream>

using namespace MultiAircraft;

bool FlightFlock::write_json_file = true;

void FlightFlock::process_time(const int t, std::list<Flock::IndexPoint> &P)
{
  auto candidates = find_candidate_disks(P);
  // sort BBs by x
  candidates.sort([](const Flock::BoundingBox &a, const Flock::BoundingBox &b)
                  { return a.ll.x <= b.ll.x; });

  auto prev = disk_store.begin();
  const bool prev_valid = prev != disk_store.end();

  disk_store.push_front(filter_candidates(candidates, t));
  auto &disks = disk_store.front();

  if (prev_valid)
  {
    connect_disks(disks, *prev);
    finalise_disks(*prev);
  }

  // set prune to true; will set to false when used next time step
  mark_all_prune(disks);
}

bool FlightFlock::find_flock(const int index) const
{
  auto it = disk_store.begin();
  if (it == disk_store.end())
    return false;
  for (auto &disk : *it)
  {
    if (disk.duration < min_duration)
      continue;
    if (disk.present(index))
      return true;
  }
  return false;
}

void FlightFlock::finalise()
{
  if (write_json_file)
  {
    std::ofstream json_file("flock.json");
    json_file << boost::json::serialize(json_records);
  }

  if (!disk_store.empty())
  {
    finalise_disks(disk_store.front());
  }
}

void FlightFlock::finalise_disks(Flock::DiskStore &disks)
{
  for (auto &d : disks)
  {
    if (!d.prune)
      continue;

    if (!d.duration)
      continue;

    std::list<Flock::DiskStore::const_iterator> chain;
    auto it = d.it_prev;
    double av = d.points.size();
    int n = 1;

    while (it->duration)
    {
      av += it->points.size();
      n++;
      chain.push_front(it);
      it = it->it_prev;
    };

    if (d.duration >= min_duration)
    {
      first_out = true;

      boost::json::array json_trace;
      int i = 0;
      for (auto &it : chain)
      {
        if (i++ % 15 == 0)
        {
          json_trace.emplace_back(report_disk(*it));
        }
      };
      json_trace.emplace_back(report_disk(d));

      boost::json::object json_disk = {
          {"duration", d.duration},
          {"av_size", av / n},
          {"bounds", {{"west", bounds.GetWest().Degrees()}, {"east", bounds.GetEast().Degrees()}, {"north", bounds.GetNorth().Degrees()}, {"south", bounds.GetSouth().Degrees()}}},
          {"trace", json_trace}};
      json_records.emplace_back(json_disk);

      n_flock++;
    }
  }
  // disks.prune_disks();
}

boost::json::object FlightFlock::report_disk(const Flock::DiskNode &d)
{
  const Flock::Point c = d.calc_center();
  const GeoPoint p = unproject_loc(c);

  if (!first_out)
  {
    bounds.Extend(p);
  }
  else
  {
    bounds = GeoBounds(p);
  }

  boost::json::array json_members;
  for (auto &i : d.points)
  {
    json_members.emplace_back(ids[i.index]);
  }

  boost::json::object json_disk = {
      {"t", d.t},
      {"lng", p.longitude.Degrees()},
      {"lat", p.latitude.Degrees()},
      {"members", json_members}};

  first_out = false;
  return json_disk;
}

const GeoPoint FlightFlock::unproject_loc(const Flock::Point &p) const
{
  return proj.Unproject(FlatPoint(p.x, p.y) / proj.GetApproximateScale());
}

void FlightFlock::mark_in_flock(std::list<AircraftModel> &aircraft) const
{
  for (auto &m : aircraft)
  {
    m.in_flock = find_flock(m.idi);
  }
}
