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

#include <string>
#include <string_view>
#include <vector>

static boost::json::value
BuildOptionsSchema()
{
  auto number_object = []()
  { return boost::json::object{{"type", "number"}}; };

  auto covariance_schema = [number_object]()
  {
    return boost::json::object{
        {"type", "object"},
        {"additionalProperties", false},
        {"properties", boost::json::object{
                           {"x", number_object()},
                           {"y", number_object()},
                           {"z", number_object()},
                           {"u", number_object()},
                           {"w", number_object()},
                           {"q", number_object()},
                           {"attitude_x", number_object()},
                           {"attitude_y", number_object()},
                           {"attitude_z", number_object()},
                           {"v_tas", number_object()},
                       }}};
  };

  return boost::json::object{
      {"$schema", "https://json-schema.org/draft/2020-12/schema"},
      {"title", "RunMultiAircraft options"},
      {"type", "object"},
      {"additionalProperties", true},
      {"properties", boost::json::object{
                         {"root", boost::json::object{{"type", "string"}}},
                         {"distance", number_object()},
                         {"score_buffer", number_object()},
                         {"height_threshold", number_object()},
                         {"velocity_scale", number_object()},
                         {"typical_trail", boost::json::object{{"type", "integer"}}},
                         {"mix_baro", number_object()},
                         {"filter_type", boost::json::object{{"type", "integer"}}},
                         {"reconstruction_pre_buffer", number_object()},
                         {"rts_window_size", boost::json::object{{"type", "integer"}}},
                         {"process_covariance", covariance_schema()},
                         {"measurement_covariance", covariance_schema()},
                         {"state_covariance", covariance_schema()},
                         {"vignette", boost::json::object{
                                          {"type", "object"},
                                          {"additionalProperties", false},
                                          {"properties", boost::json::object{
                                                             {"subject", boost::json::object{{"type", "string"}}},
                                                             {"start_time", boost::json::object{{"type", "integer"}}},
                                                             {"end_time", boost::json::object{{"type", "integer"}}},
                                                         }},
                                          {"required", boost::json::array{"subject", "start_time", "end_time"}},
                                      }},
                         {"igc_files", boost::json::object{
                                           {"type", "array"},
                                           {"items", boost::json::object{{"type", "string"}}},
                                       }},
                     }},
      {"required", boost::json::array{"igc_files"}},
  };
}

static void
WriteOptionsSchema(const Path &path)
{
  std::ofstream file(path.c_str());
  file << boost::json::serialize(BuildOptionsSchema()) << "\n";
}

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

static void
ApplyCovarianceOverrides(const boost::json::object &j,
                         const char *key,
                         bool (*setter)(std::string_view, double))
{
  try
  {
    const auto &overrides = j.at(key).as_object();
    for (const auto &entry : overrides)
    {
      if (!entry.value().is_number())
      {
        std::cout << "Ignoring non-numeric " << key
                  << " value for '" << entry.key() << "'\n";
        continue;
      }

      const double value = entry.value().to_number<double>();
      if (!setter(entry.key(), value))
      {
        std::cout << "Ignoring unknown " << key
                  << " name '" << entry.key() << "'\n";
      }
    }
  }
  catch (const boost::system::system_error &e)
  {
  }
}

static void DecodeOptions(const boost::json::value &_j,
                          MultiAircraft::FlightCollectionEncounter &flights,
                          VignetteConfig &vignette)
{
  const auto &j = _j.as_object();
  auto try_apply = [&](const char *key, auto &&fn)
  {
    try
    {
      fn(j.at(key));
      return true;
    }
    catch (const boost::system::system_error &)
    {
      return false;
    }
  };

  auto try_apply_report = [&](const char *key, auto &&fn)
  {
    try
    {
      fn(j.at(key));
      return true;
    }
    catch (const boost::system::system_error &e)
    {
      std::cout << e.what() << std::endl;
      return false;
    }
  };

  std::string root;
  if (!try_apply("root", [&](const boost::json::value &v)
                 { root = std::string(v.as_string()); }))
  {
    const char *root_env = std::getenv("ROOT");
    if (root_env != nullptr)
      root = std::string(root_env);
  }

  if (!root.empty())
  {
    const std::string geoid_file = root + "/igc_fr_geoid.txt";
    IGCFRInfoDB_init(geoid_file.c_str());
  }

  try_apply_report("distance", [&](const boost::json::value &v)
                   { flights.DISTANCE = v.to_number<double>(); });
  try_apply_report("score_buffer", [&](const boost::json::value &v)
                   { flights.SCORE_BUFFER = v.to_number<double>(); });

  try_apply_report("height_threshold", [&](const boost::json::value &v)
                   {
                     flights.HEIGHT_THRESHOLD_M = v.to_number<double>();
                     std::cout << "height_threshold\n"; });

  try_apply_report("velocity_scale", [&](const boost::json::value &v)
                   {
                     MultiAircraft::DetectMiss::VELOCITY_SCALE_MS =
                         v.to_number<double>();
                     std::cout << "velocity_scale\n"; });

  try_apply("typical_trail", [&](const boost::json::value &v)
            {
              MultiAircraft::EncounterMapStore::TYP_TRAIL = v.as_int64();
              std::cout << "typical_trail\n"; });

  try_apply("mix_baro", [&](const boost::json::value &v)
            { MultiAircraft::AircraftModel::MIX_BARO = v.to_number<double>(); });

  try_apply_report("filter_type", [&](const boost::json::value &v)
                   { MultiAircraft::AircraftModel::filter_type =
                         v.to_number<unsigned>(); });

  try_apply("reconstruction_pre_buffer", [&](const boost::json::value &v)
            { MultiAircraft::AircraftModel::SetReconstructionPreBuffer(
                  v.to_number<double>()); });

  ApplyCovarianceOverrides(
      j, "process_covariance",
      &FlightReconstruction::SetProcessCovarianceDefault);
  ApplyCovarianceOverrides(
      j, "measurement_covariance",
      &FlightReconstruction::SetMeasurementCovarianceDefault);
  ApplyCovarianceOverrides(
      j, "state_covariance",
      &FlightReconstruction::SetStateCovarianceDefault);

  try_apply("rts_window_size", [&](const boost::json::value &v)
            {
              const unsigned rts_window_size = v.to_number<unsigned>();
              if (MultiAircraft::AircraftModel::filter_type == 1 ||
                  MultiAircraft::AircraftModel::filter_type == 2)
              {
                FlightReconstruction::SetRTSWindowSize(rts_window_size);
              }
              else
              {
                FlightReconstruction::SetRTSWindowSize(0);
                std::cout << "Ignoring rts_window_size for filter_type="
                          << MultiAircraft::AircraftModel::filter_type
                          << " (valid only for 1 or 2)\n";
              } });

  try_apply("vignette", [&](const boost::json::value &v)
            {
              const auto &obj = v.as_object();
              vignette.subject = std::string(obj.at("subject").as_string());
              vignette.start_time = obj.at("start_time").to_number<unsigned>();
              vignette.end_time = obj.at("end_time").to_number<unsigned>();
              if (vignette.end_time < vignette.start_time)
              {
                throw std::invalid_argument{
                    "vignette.end_time < vignette.start_time"};
              }
              vignette.enabled = true; });

  try
  {
    const auto &igc_file_array = j.at("igc_files").as_array();
    const size_t n = igc_file_array.size();
    if (n > 0)
    {
      std::vector<std::string> file_storage;
      file_storage.reserve(n);
      std::vector<const char *> files;
      files.reserve(n + 1);
      files.push_back("ignored");

      for (const auto &entry : igc_file_array)
      {
        file_storage.emplace_back(entry.as_string());
        files.push_back(file_storage.back().c_str());
      }

      Args file_args(n + 1, const_cast<char **>(files.data()), "igc_files");
      if (flights.load_files(file_args))
      {
        return;
      }

      throw std::invalid_argument{"Can't load igc files"};
    }
  }
  catch (const boost::system::system_error &)
  {
  }

  throw std::invalid_argument{"insufficient igc files"};
}

int main(int argc, char **argv)
{
  MultiAircraft::FlightCollectionEncounter flights;
  VignetteConfig vignette;

  Args args(argc, argv, "options.json");
  Path path = Path("options.json");
  const bool has_json_arg =
      !args.IsEmpty() && StringEndsWithIgnoreCase(args.PeekNext(), ".json");
  if (has_json_arg)
  {
    path = args.ExpectNextPath();
  }

  boost::json::value json_data;
  try
  {
    json_data = ParseJsonFile(path);
  }
  catch (const std::exception &e)
  {
    if (!has_json_arg)
    {
      const Path schema_path = Path("RunMultiAircraft.schema.json");
      WriteOptionsSchema(schema_path);
      std::cout << "No input JSON found at '" << path.c_str() << "'.\n"
                << "Wrote schema to '" << schema_path.c_str() << "'.\n";
      return 0;
    }

    throw;
  }

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
