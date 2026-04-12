// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "system/Args.hpp"
#include <stdio.h>
#include "MultiAircraft/FlightCollectionEncounter.hpp"
#include "MultiAircraft/FlightReconstructionOptions.hpp"
#include "IGC/IGCFRInfo.hpp"
#include "io/FileReader.hxx"
#include "json/Parse.hxx"
#include "util/StringCompare.hxx"

bool debug = false;

struct VignetteConfig
{
  bool enabled = false;
  std::string subject;
  unsigned start_time = 0;
  unsigned end_time = 0;
};

static auto
ParseJsonFile(Path path)
{
  FileReader r{path};
  return Json::Parse(r);
}

static void DecodeOptions(const boost::json::value &_j,
                          MultiAircraft::FlightCollectionEncounter &flights,
                          VignetteConfig &vignette)
{
  const auto &j = _j.as_object();
  std::string root;

  try
  {
    root = j.at("root").as_string();
  }
  catch (const boost::system::system_error &e)
  {
    const char *root_env = std::getenv("ROOT");
    if (root_env != nullptr)
    {
      root = std::string(root_env);
    }
  }

  if (!root.empty())
  {
    std::string name = root + "/igc_fr_geoid.txt";
    IGCFRInfoDB_init(name.c_str());
  }

  try
  {
    flights.DISTANCE = j.at("distance").to_number<double>();
  }
  catch (const boost::system::system_error &e)
  {
    std::cout << e.what() << std::endl;
  }

  try
  {
    flights.SCORE_BUFFER = j.at("score_buffer").to_number<double>();
  }
  catch (const boost::system::system_error &e)
  {
    std::cout << e.what() << std::endl;
  }

  try
  {
    flights.HEIGHT_THRESHOLD_M = j.at("height_threshold").to_number<double>();
    std::cout << "height_threshold\n";
  }
  catch (const boost::system::system_error &e)
  {
    std::cout << e.what() << std::endl;
  }

  try
  {
    MultiAircraft::DetectMiss::VELOCITY_SCALE_MS = j.at("velocity_scale").to_number<double>();
    std::cout << "velocity_scale\n";
  }
  catch (const boost::system::system_error &e)
  {
    std::cout << e.what() << std::endl;
  }

  try
  {
    MultiAircraft::EncounterMapStore::TYP_TRAIL = j.at("typical_trail").as_int64();
    std::cout << "typical_trail\n";
  }
  catch (const boost::system::system_error &e)
  {
  }

  try
  {
    MultiAircraft::AircraftModel::MIX_BARO = j.at("mix_baro").to_number<double>();
  }
  catch (const boost::system::system_error &e)
  {
  }

  try
  {
    MultiAircraft::AircraftModel::filter_type = j.at("filter_type").to_number<unsigned>();
  }
  catch (const boost::system::system_error &e)
  {
    std::cout << e.what() << std::endl;
  }

  try
  {
    FlightReconstruction::SetRTSWindowSize(j.at("rts_window_size").to_number<unsigned>());
  }
  catch (const boost::system::system_error &e)
  {
  }

  try
  {
    const auto &v = j.at("vignette").as_object();
    vignette.subject = std::string(v.at("subject").as_string());
    vignette.start_time = v.at("start_time").to_number<unsigned>();
    vignette.end_time = v.at("end_time").to_number<unsigned>();
    if (vignette.end_time < vignette.start_time)
    {
      throw std::invalid_argument{"vignette.end_time < vignette.start_time"};
    }
    vignette.enabled = true;
  }
  catch (const boost::system::system_error &e)
  {
  }

  try
  {
    auto igc_file_array = j.at("igc_files").as_array();
    const size_t n = igc_file_array.size();
    if (n > 0)
    {
      typedef const char *ccs;
      ccs *files = new ccs[n + 1];
      const char *ignored = "ignored";
      files[0] = ignored;
      for (size_t i = 0; i < n; ++i)
      {
        files[i + 1] = igc_file_array.at(i).as_string().c_str();
      }
      auto args = Args(n + 1, const_cast<char **>(files), "igc_files");
      delete[] files;
      if (flights.load_files(args))
      {
        return;
      }
      else
      {
        throw std::invalid_argument{"Can't load igc files"};
        exit(EXIT_FAILURE);
      }
    }
  }
  catch (const boost::system::system_error &e)
  {
  }

  throw std::invalid_argument{"insufficient igc files"};
  exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
  MultiAircraft::FlightCollectionEncounter flights;
  VignetteConfig vignette;

  Args args(argc, argv, "options.json");
  Path path = Path("options.json");
  if (!args.IsEmpty() && StringEndsWithIgnoreCase(args.PeekNext(), ".json"))
  {
    path = args.ExpectNextPath();
  }
  auto json_data = ParseJsonFile(path);
  DecodeOptions(json_data, flights, vignette);
  MultiAircraft::AircraftModel::SetWriteTraceFiles(!vignette.enabled);
  MultiAircraft::AircraftModel::SetKeepFullTrail(vignette.enabled);
  MultiAircraft::FlightFlock::SetWriteJsonFile(!vignette.enabled);
  if (vignette.enabled)
  {
    MultiAircraft::FlightCollectionEncounter::VignetteOptions options;
    options.subject = vignette.subject;
    options.start_time = vignette.start_time;
    options.end_time = vignette.end_time;
    flights.SetVignetteOptions(options);
  }
  args.ExpectEnd();

  flights.run();

  if (vignette.enabled)
  {
    exit(0);
  }

  {
    std::ofstream file_summary("summary.json");
    boost::json::object summary = flights.record_summary();
    file_summary << boost::json::serialize(summary);
  }

  exit(0);
}
