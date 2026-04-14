// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "system/Args.hpp"
#include <stdio.h>
#include "MultiAircraft/CovarianceTuner.hpp"
#include "MultiAircraft/FlightCollectionEncounter.hpp"
#include "MultiAircraft/FlightReconstructionOptions.hpp"
#include "IGC/IGCFRInfo.hpp"
#include "io/FileReader.hxx"
#include "json/Parse.hxx"
#include "util/StringCompare.hxx"

#include <fstream>
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
                           {"w_g", number_object()},
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
                         {"covariance_tuning", boost::json::object{
                                                   {"type", "object"},
                                                   {"additionalProperties", false},
                                                   {"properties", boost::json::object{
                                                                      {"enabled", boost::json::object{{"type", "boolean"}}},
                                                                      {"output", boost::json::object{{"type", "string"}}},
                                                                      {"min_samples", boost::json::object{{"type", "integer"}}},
                                                                      {"blend", number_object()},
                                                                      {"min_diagonal", number_object()},
                                                                      {"max_flights", boost::json::object{{"type", "integer"}}},
                                                                      {"max_points_per_flight", boost::json::object{{"type", "integer"}}},
                                                                      {"max_failures_per_flight", boost::json::object{{"type", "integer"}}},
                                                                      {"max_consecutive_failures", boost::json::object{{"type", "integer"}}},
                                                                      {"max_restarts_per_flight", boost::json::object{{"type", "integer"}}},
                                                                      {"bootstrap_iterations", boost::json::object{{"type", "integer"}}},
                                                                      {"bootstrap_fraction", number_object()},
                                                                      {"random_seed", boost::json::object{{"type", "integer"}}},
                                                                      {"convergence_enabled", boost::json::object{{"type", "boolean"}}},
                                                                      {"convergence_min_passes", boost::json::object{{"type", "integer"}}},
                                                                      {"convergence_max_passes", boost::json::object{{"type", "integer"}}},
                                                                      {"convergence_flights_per_pass", boost::json::object{{"type", "integer"}}},
                                                                      {"convergence_rel_tolerance", number_object()},
                                                                  }},
                                               }},
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
                         {"airspace", boost::json::object{
                                          {"type", "object"},
                                          {"additionalProperties", false},
                                          {"properties", boost::json::object{
                                                             {"openaip", boost::json::object{{"type", "string"}}},
                                                         }},
                                          {"required", boost::json::array{"openaip"}},
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

struct AirspaceConfig
{
  std::string openaip_path;

  bool enabled() const noexcept
  {
    return !openaip_path.empty();
  }
};

static void
DecodeCovarianceTuning(const boost::json::object &obj,
                       MultiAircraft::CovarianceTuningConfig &config)
{
  auto try_apply = [&](const char *key, auto &&fn)
  {
    try
    {
      fn(obj.at(key));
    }
    catch (const boost::system::system_error &)
    {
    }
  };

  try_apply("enabled", [&](const boost::json::value &v)
            { config.enabled = v.as_bool(); });
  try_apply("output", [&](const boost::json::value &v)
            { config.output_path = std::string(v.as_string()); });
  try_apply("min_samples", [&](const boost::json::value &v)
            { config.min_samples = v.to_number<std::size_t>(); });
  try_apply("blend", [&](const boost::json::value &v)
            { config.blend = v.to_number<double>(); });
  try_apply("min_diagonal", [&](const boost::json::value &v)
            { config.min_diagonal = v.to_number<double>(); });
  try_apply("max_flights", [&](const boost::json::value &v)
            { config.max_flights = v.to_number<std::size_t>(); });
  try_apply("max_points_per_flight", [&](const boost::json::value &v)
            { config.max_points_per_flight = v.to_number<std::size_t>(); });
  try_apply("max_failures_per_flight", [&](const boost::json::value &v)
            { config.max_failures_per_flight = v.to_number<std::size_t>(); });
  try_apply("max_consecutive_failures", [&](const boost::json::value &v)
            { config.max_consecutive_failures = v.to_number<std::size_t>(); });
  try_apply("max_restarts_per_flight", [&](const boost::json::value &v)
            { config.max_restarts_per_flight = v.to_number<std::size_t>(); });
  try_apply("bootstrap_iterations", [&](const boost::json::value &v)
            { config.bootstrap_iterations = v.to_number<std::size_t>(); });
  try_apply("bootstrap_fraction", [&](const boost::json::value &v)
            { config.bootstrap_fraction = v.to_number<double>(); });
  try_apply("random_seed", [&](const boost::json::value &v)
            { config.random_seed = v.to_number<unsigned>(); });
  try_apply("convergence_enabled", [&](const boost::json::value &v)
            { config.convergence_enabled = v.as_bool(); });
  try_apply("convergence_min_passes", [&](const boost::json::value &v)
            { config.convergence_min_passes = v.to_number<std::size_t>(); });
  try_apply("convergence_max_passes", [&](const boost::json::value &v)
            { config.convergence_max_passes = v.to_number<std::size_t>(); });
  try_apply("convergence_flights_per_pass", [&](const boost::json::value &v)
            { config.convergence_flights_per_pass = v.to_number<std::size_t>(); });
  try_apply("convergence_rel_tolerance", [&](const boost::json::value &v)
            { config.convergence_rel_tolerance = v.to_number<double>(); });
}

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
                          VignetteConfig &vignette,
                          AirspaceConfig &airspace,
                          MultiAircraft::CovarianceTuningConfig &covariance_tuning)
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

  const auto uses_updraft_gust_filter = []()
  {
    const int filter_type = MultiAircraft::AircraftModel::filter_type;
    return filter_type == 3 || filter_type == 4;
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
                   { flights.HEIGHT_THRESHOLD_M = v.to_number<double>(); });

  try_apply_report("velocity_scale", [&](const boost::json::value &v)
                   { MultiAircraft::DetectMiss::VELOCITY_SCALE_MS =
                         v.to_number<double>(); });

  try_apply("typical_trail", [&](const boost::json::value &v)
            { MultiAircraft::EncounterMapStore::TYP_TRAIL = v.as_int64(); });

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
      uses_updraft_gust_filter()
          ? &FlightReconstruction::SetProcessCovarianceDefaultWithUpdraftGust
          : &FlightReconstruction::SetProcessCovarianceDefault);
  ApplyCovarianceOverrides(
      j, "measurement_covariance",
      &FlightReconstruction::SetMeasurementCovarianceDefault);
  ApplyCovarianceOverrides(
      j, "state_covariance",
      uses_updraft_gust_filter()
          ? &FlightReconstruction::SetStateCovarianceDefaultWithUpdraftGust
          : &FlightReconstruction::SetStateCovarianceDefault);

  try_apply("rts_window_size", [&](const boost::json::value &v)
            {
              const unsigned rts_window_size = v.to_number<unsigned>();
              if (MultiAircraft::AircraftModel::filter_type >= 1 &&
                  MultiAircraft::AircraftModel::filter_type <= 4)
              {
                FlightReconstruction::SetRTSWindowSize(rts_window_size);
              }
              else
              {
                FlightReconstruction::SetRTSWindowSize(0);
                std::cout << "Ignoring rts_window_size for filter_type="
                          << MultiAircraft::AircraftModel::filter_type
                          << " (valid only for 1..4)\n";
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

  try_apply("airspace", [&](const boost::json::value &v)
            {
              const auto &obj = v.as_object();
              airspace.openaip_path = std::string(obj.at("openaip").as_string()); });

  try_apply("covariance_tuning", [&](const boost::json::value &v)
            { DecodeCovarianceTuning(v.as_object(), covariance_tuning); });

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
  AirspaceConfig airspace;
  MultiAircraft::CovarianceTuningConfig covariance_tuning;

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

  DecodeOptions(json_data, flights, vignette, airspace, covariance_tuning);
  MultiAircraft::AircraftModel::SetWriteTraceFiles(!vignette.enabled);
  MultiAircraft::AircraftModel::SetKeepFullTrail(vignette.enabled || airspace.enabled() || covariance_tuning.enabled);
  MultiAircraft::FlightCollectionEncounter::SetSkipEncounterProcessing(covariance_tuning.enabled);
  MultiAircraft::FlightFlock::SetWriteJsonFile(!vignette.enabled);
  if (airspace.enabled())
    flights.LoadOpenAipAirspaces(Path(airspace.openaip_path.c_str()));
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

  if (covariance_tuning.enabled)
  {
    std::cout << "[cov-tune] invoke output='" << covariance_tuning.output_path
              << "' min_samples=" << covariance_tuning.min_samples
              << " blend=" << covariance_tuning.blend
              << " min_diagonal=" << covariance_tuning.min_diagonal
              << " max_flights=" << covariance_tuning.max_flights
              << " max_points_per_flight=" << covariance_tuning.max_points_per_flight
              << " max_failures_per_flight=" << covariance_tuning.max_failures_per_flight
              << " max_consecutive_failures=" << covariance_tuning.max_consecutive_failures
              << " max_restarts_per_flight=" << covariance_tuning.max_restarts_per_flight
              << " bootstrap_iterations=" << covariance_tuning.bootstrap_iterations
              << " bootstrap_fraction=" << covariance_tuning.bootstrap_fraction
              << " random_seed=" << covariance_tuning.random_seed
              << " convergence_enabled=" << covariance_tuning.convergence_enabled
              << " convergence_min_passes=" << covariance_tuning.convergence_min_passes
              << " convergence_max_passes=" << covariance_tuning.convergence_max_passes
              << " convergence_flights_per_pass=" << covariance_tuning.convergence_flights_per_pass
              << " convergence_rel_tolerance=" << covariance_tuning.convergence_rel_tolerance
              << "\n";

    const auto tuned = MultiAircraft::TuneFlightReconstructionCovariances(
        flights.GetAircraft(), covariance_tuning);

    if (!tuned.success)
    {
      std::cout << "Covariance tuning skipped: " << tuned.message
                << " (measurement=" << tuned.measurement_samples
                << ", process=" << tuned.process_samples
                << ", initial_state=" << tuned.initial_state_samples << ")\n";
    }
    else
    {
      boost::json::object out;
      out.emplace("covariance_tuning", tuned.covariances);

      std::ofstream file(covariance_tuning.output_path);
      file << boost::json::serialize(out) << "\n";
      std::cout << "[cov-tune] wrote tuned covariance file: " << covariance_tuning.output_path << "\n";
    }
  }

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
