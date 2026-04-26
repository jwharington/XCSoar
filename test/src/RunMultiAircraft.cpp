// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "system/Args.hpp"
#include <stdio.h>
#include "MultiAircraft/AirfieldList.hpp"
#include "MultiAircraft/CovarianceTuner.hpp"
#include "MultiAircraft/FlightCollectionEncounter.hpp"
#include "MultiAircraft/FlightReconstructionOptions.hpp"
#include "MultiAircraft/TrajectoryTokenizerQTC3D.hpp"
#include "MultiAircraft/TrajectoryTokenizerTokenOps.hpp"
#include "IGC/IGCFRInfo.hpp"
#include "io/FileReader.hxx"
#include "json/Parse.hxx"
#include "util/StringCompare.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
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
                         {"terrain_clearance", number_object()},
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
                                                             {"incursion_threshold", boost::json::object{{"type", "number"}}},
                                                         }},
                                          {"required", boost::json::array{"openaip"}},
                                      }},
                         {"terrain", boost::json::object{
                                         {"type", "object"},
                                         {"additionalProperties", false},
                                         {"properties", boost::json::object{
                                                            {"files", boost::json::object{
                                                                          {"type", "array"},
                                                                          {"items", boost::json::object{{"type", "string"}}},
                                                                      }},
                                                        }},
                                         {"required", boost::json::array{"files"}},
                                     }},
                         {"airfields", boost::json::object{
                                           {"type", "array"},
                                           {"items", boost::json::object{
                                                         {"type", "object"},
                                                         {"additionalProperties", false},
                                                         {"properties", boost::json::object{
                                                                            {"latitude", number_object()},
                                                                            {"longitude", number_object()},
                                                                            {"name", boost::json::object{{"type", "string"}}},
                                                                        }},
                                                         {"required", boost::json::array{"latitude", "longitude"}},
                                                     }},
                                       }},
                         {"trajectory_tokenizer", boost::json::object{
                                                     {"type", "object"},
                                                     {"additionalProperties", false},
                                                     {"properties", boost::json::object{
                                                                        {"enabled", boost::json::object{{"type", "boolean"}}},
                                                                        {"subject", boost::json::object{{"type", "string"}}},
                                                                        {"output", boost::json::object{{"type", "string"}}},
                                                                        {"max_time_delta_seconds", number_object()},
                                                                        {"min_dt_seconds", number_object()},
                                                                        {"use_baro_altitude", boost::json::object{{"type", "boolean"}}},
                                                                        {"include_implausible", boost::json::object{{"type", "boolean"}}},
                                                                        {"radial_zero_m", number_object()},
                                                                        {"vertical_zero_m", number_object()},
                                                                        {"lateral_zero_rad", number_object()},
                                                                        {"use_custom_levels", boost::json::object{{"type", "boolean"}}},
                                                                        {"normalize_rates", boost::json::object{{"type", "boolean"}}},
                                                                        {"nonuniform_bins", boost::json::object{{"type", "boolean"}}},
                                                                        {"nonuniform_bin_exponent", number_object()},
                                                                        {"closing_only_until_t0", boost::json::object{{"type", "boolean"}}},
                                                                        {"closing_t0_seconds", number_object()},
                                                                        {"radial_levels", boost::json::object{{"type", "integer"}}},
                                                                        {"vertical_levels", boost::json::object{{"type", "integer"}}},
                                                                        {"lateral_levels", boost::json::object{{"type", "integer"}}},
                                                                        {"bank_levels", boost::json::object{{"type", "integer"}}},
                                                                        {"dbank_levels", boost::json::object{{"type", "integer"}}},
                                                                        {"radial_min", number_object()},
                                                                        {"radial_max", number_object()},
                                                                        {"vertical_min", number_object()},
                                                                        {"vertical_max", number_object()},
                                                                        {"lateral_min", number_object()},
                                                                        {"lateral_max", number_object()},
                                                                        {"bank_min_deg", number_object()},
                                                                        {"bank_max_deg", number_object()},
                                                                        {"dbank_min_deg", number_object()},
                                                                        {"dbank_max_deg", number_object()},
                                                                        {"fit_mad_thresholds", boost::json::object{{"type", "boolean"}}},
                                                                        {"mad_fit_scale", number_object()},
                                                                        {"mad_fit_min_encounters", boost::json::object{{"type", "integer"}}},
                                                                        {"mad_fit_max_encounters", boost::json::object{{"type", "integer"}}},
                                                                        {"mad_fit_check_interval", boost::json::object{{"type", "integer"}}},
                                                                        {"mad_fit_rel_tolerance", number_object()},
                                                                        {"mad_fit_patience", boost::json::object{{"type", "integer"}}},
                                                                        {"mad_fit_min_radial_zero_m", number_object()},
                                                                        {"mad_fit_min_vertical_zero_m", number_object()},
                                                                        {"mad_fit_min_lateral_zero_rad", number_object()},
                                                                        {"include_bank_state", boost::json::object{{"type", "boolean"}}},
                                                                        {"bank_zero_deg", number_object()},
                                                                        {"include_bank_trend", boost::json::object{{"type", "boolean"}}},
                                                                        {"bank_delta_zero_deg", number_object()},
                                                                        {"canonicalize_pair_order", boost::json::object{{"type", "boolean"}}},
                                                                        {"handedness_invariant", boost::json::object{{"type", "boolean"}}},
                                                                        {"reject_near_identical_pair", boost::json::object{{"type", "boolean"}}},
                                                                        {"pair_identical_xy_tol_m", number_object()},
                                                                        {"pair_identical_alt_tol_m", number_object()},
                                                                        {"pair_identical_min_fraction", number_object()},
                                                                        {"rle_encode", boost::json::object{{"type", "boolean"}}},
                                                                        {"include_raw_tokens", boost::json::object{{"type", "boolean"}}},
                                                                        {"max_tokens_per_pair", boost::json::object{{"type", "integer"}}},
                                                                    }},
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
  double incursion_threshold = 50;

  bool enabled() const noexcept
  {
    return !openaip_path.empty();
  }
};

struct TrajectoryTokenizerConfig
{
  bool enabled = false;
  std::string subject;
  std::string output = "trajectory_tokens.qtc3d.json";
  double max_time_delta_seconds = 1.0;
  double min_dt_seconds = 0.4;
  bool use_baro_altitude = true;
  bool include_implausible = false;
  double radial_zero_m = 0.5;
  double vertical_zero_m = 0.2;
  double lateral_zero_rad = 1e-3;
  bool use_custom_levels = false;
  bool normalize_rates = true;
  bool nonuniform_bins = false;
  double nonuniform_bin_exponent = 0.6;
  bool closing_only_until_t0 = true;
  double closing_t0_seconds = 0.0;
  unsigned radial_levels = 3;
  unsigned vertical_levels = 3;
  unsigned lateral_levels = 3;
  unsigned bank_levels = 3;
  unsigned dbank_levels = 3;
  double radial_min = -0.5;
  double radial_max = 0.5;
  double vertical_min = -0.2;
  double vertical_max = 0.2;
  double lateral_min = -1e-3;
  double lateral_max = 1e-3;
  double bank_min_deg = -45.0;
  double bank_max_deg = 45.0;
  double dbank_min_deg = -10.0;
  double dbank_max_deg = 10.0;
  bool fit_mad_thresholds = false;
  double mad_fit_scale = 1.0;
  unsigned mad_fit_min_encounters = 100;
  unsigned mad_fit_max_encounters = 0; // 0 = all
  unsigned mad_fit_check_interval = 50;
  double mad_fit_rel_tolerance = 0.01;
  unsigned mad_fit_patience = 3;
  double mad_fit_min_radial_zero_m = 0.05;
  double mad_fit_min_vertical_zero_m = 0.05;
  double mad_fit_min_lateral_zero_rad = 1e-4;
  bool include_bank_state = true;
  double bank_zero_deg = 5.0;
  bool include_bank_trend = true;
  double bank_delta_zero_deg = 1.0;
  bool canonicalize_pair_order = true;
  bool handedness_invariant = false;
  bool reject_near_identical_pair = true;
  double pair_identical_xy_tol_m = 1.0;
  double pair_identical_alt_tol_m = 1.0;
  double pair_identical_min_fraction = 0.98;
  bool rle_encode = false;
  bool include_raw_tokens = true;
  unsigned max_tokens_per_pair = 0; // 0 = unlimited
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

static double
WrapAngleRad(double x)
{
  while (x > M_PI)
    x -= 2.0 * M_PI;
  while (x < -M_PI)
    x += 2.0 * M_PI;
  return x;
}

static std::string
AircraftIdentityKey(const boost::json::object &o)
{
  const auto it_id = o.find("id");
  if (it_id != o.end() && it_id->value().is_string())
    return std::string(it_id->value().as_string());

  const auto it_fr = o.find("fr_id");
  if (it_fr != o.end() && it_fr->value().is_string())
    return std::string(it_fr->value().as_string());

  return {};
}

static bool
AreTracesNearlyIdentical(const boost::json::object &a,
                         const boost::json::object &b,
                         const bool use_baro_altitude,
                         const double xy_tol_m,
                         const double alt_tol_m,
                         const double min_fraction)
{
  const auto ita = a.find("trace");
  const auto itb = b.find("trace");
  if (ita == a.end() || itb == b.end() ||
      !ita->value().is_array() || !itb->value().is_array())
    return false;

  const auto &ta = ita->value().as_array();
  const auto &tb = itb->value().as_array();
  const std::size_t n = std::min<std::size_t>(ta.size(), tb.size());
  if (n < 2)
    return false;

  const char *alt_key = use_baro_altitude ? "alt_baro" : "alt_gps";

  std::size_t compared = 0;
  std::size_t near = 0;
  for (std::size_t k = 0; k < n; ++k)
  {
    if (!ta[k].is_object() || !tb[k].is_object())
      continue;

    const auto &pa = ta[k].as_object();
    const auto &pb = tb[k].as_object();

    const auto req = [](const boost::json::object &o, const char *key)
    { return o.find(key) != o.end() && o.at(key).is_number(); };

    if (!(req(pa, "x") && req(pa, "y") && req(pa, alt_key) &&
          req(pb, "x") && req(pb, "y") && req(pb, alt_key)))
      continue;

    ++compared;

    const double dx = pa.at("x").to_number<double>() - pb.at("x").to_number<double>();
    const double dy = pa.at("y").to_number<double>() - pb.at("y").to_number<double>();
    const double dxy = std::hypot(dx, dy);
    const double dz = std::abs(pa.at(alt_key).to_number<double>() - pb.at(alt_key).to_number<double>());

    if (dxy <= xy_tol_m && dz <= alt_tol_m)
      ++near;
  }

  if (compared < 8)
    return false;

  const double fraction = static_cast<double>(near) / static_cast<double>(compared);
  return fraction >= std::clamp(min_fraction, 0.0, 1.0);
}

static bool
ComputePairMinDistance2D(const boost::json::object &a,
                         const boost::json::object &b,
                         double &out_min_distance)
{
  const auto ita = a.find("trace");
  const auto itb = b.find("trace");
  if (ita == a.end() || itb == b.end() ||
      !ita->value().is_array() || !itb->value().is_array())
    return false;

  const auto &ta = ita->value().as_array();
  const auto &tb = itb->value().as_array();
  const std::size_t n = std::min<std::size_t>(ta.size(), tb.size());
  if (n < 2)
    return false;

  bool have = false;
  double best = std::numeric_limits<double>::infinity();

  for (std::size_t k = 0; k < n; ++k)
  {
    if (!ta[k].is_object() || !tb[k].is_object())
      continue;

    const auto &pa = ta[k].as_object();
    const auto &pb = tb[k].as_object();
    const auto req = [](const boost::json::object &o, const char *key)
    { return o.find(key) != o.end() && o.at(key).is_number(); };

    if (!(req(pa, "x") && req(pa, "y") && req(pb, "x") && req(pb, "y")))
      continue;

    const double dx = pa.at("x").to_number<double>() - pb.at("x").to_number<double>();
    const double dy = pa.at("y").to_number<double>() - pb.at("y").to_number<double>();
    const double d = std::hypot(dx, dy);
    if (d < best)
      best = d;
    have = true;
  }

  if (!have)
    return false;

  out_min_distance = best;
  return true;
}

static bool
SelectDistinctAircraftPair(const boost::json::array &aircraft,
                           const TrajectoryTokenizerConfig &cfg,
                           std::size_t &ia,
                           std::size_t &ib)
{
  if (aircraft.size() < 2)
    return false;

  bool found = false;
  double best_min_distance = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < aircraft.size(); ++i)
  {
    if (!aircraft[i].is_object())
      continue;

    const auto &a = aircraft[i].as_object();
    const auto key_a = AircraftIdentityKey(a);

    for (std::size_t j = i + 1; j < aircraft.size(); ++j)
    {
      if (!aircraft[j].is_object())
        continue;

      const auto &b = aircraft[j].as_object();
      const auto key_b = AircraftIdentityKey(b);

      if (!key_a.empty() && !key_b.empty() && key_a == key_b)
        continue;

      const auto ita = a.find("trace");
      const auto itb = b.find("trace");
      if (ita == a.end() || itb == b.end() ||
          !ita->value().is_array() || !itb->value().is_array())
        continue;

      const auto na = ita->value().as_array().size();
      const auto nb = itb->value().as_array().size();
      if (std::min(na, nb) < 2)
        continue;

      if (cfg.reject_near_identical_pair &&
          AreTracesNearlyIdentical(a, b,
                                   cfg.use_baro_altitude,
                                   cfg.pair_identical_xy_tol_m,
                                   cfg.pair_identical_alt_tol_m,
                                   cfg.pair_identical_min_fraction))
        continue;

      double min_distance = std::numeric_limits<double>::infinity();
      if (!ComputePairMinDistance2D(a, b, min_distance))
        continue;

      if (!found || min_distance < best_min_distance)
      {
        found = true;
        best_min_distance = min_distance;
        ia = i;
        ib = j;
      }
    }
  }

  return found;
}

static double
Median(std::vector<double> values)
{
  if (values.empty())
    return 0;

  const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  if (values.size() % 2 == 1)
    return *mid;

  const double hi = *mid;
  const auto mid2 = std::max_element(values.begin(), mid);
  return (hi + *mid2) * 0.5;
}

static double
RobustSigmaFromMAD(const std::vector<double> &samples)
{
  if (samples.empty())
    return 0;

  const double med = Median(samples);
  std::vector<double> abs_dev;
  abs_dev.reserve(samples.size());
  for (const double v : samples)
    abs_dev.push_back(std::abs(v - med));

  const double mad = Median(std::move(abs_dev));
  return 1.4826 * mad;
}

struct MADFitThresholds
{
  bool fitted = false;
  std::size_t encounters_used = 0;
  std::size_t samples = 0;
  double radial_zero_m = 0;
  double vertical_zero_m = 0;
  double lateral_zero_rad = 0;
};

static MADFitThresholds
FitMADThresholdsFromEncounterFiles(const TrajectoryTokenizerConfig &cfg)
{
  MADFitThresholds out;

  std::vector<std::filesystem::path> files;
  for (const auto &entry : std::filesystem::directory_iterator(std::filesystem::current_path()))
  {
    if (!entry.is_regular_file())
      continue;

    const auto name = entry.path().filename().string();
    if (name.rfind("encounter_", 0) == 0 && entry.path().extension() == ".json")
      files.push_back(entry.path());
  }
  std::sort(files.begin(), files.end());

  if (files.empty())
    return out;

  std::vector<double> dr_samples;
  std::vector<double> dz_samples;
  std::vector<double> dl_samples;

  const std::size_t check_interval = std::max<std::size_t>(1, cfg.mad_fit_check_interval);
  const std::size_t min_encounters = std::max<std::size_t>(1, cfg.mad_fit_min_encounters);
  const std::size_t max_encounters = cfg.mad_fit_max_encounters == 0
                                         ? files.size()
                                         : std::min<std::size_t>(cfg.mad_fit_max_encounters, files.size());
  const std::size_t patience = std::max<std::size_t>(1, cfg.mad_fit_patience);

  double prev_r = std::numeric_limits<double>::quiet_NaN();
  double prev_z = std::numeric_limits<double>::quiet_NaN();
  double prev_l = std::numeric_limits<double>::quiet_NaN();
  std::size_t stable_rounds = 0;

  for (std::size_t i = 0; i < max_encounters; ++i)
  {
    boost::system::error_code ec;
    const auto json_text = [&]()
    {
      std::ifstream in(files[i]);
      return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    }();

    const auto value = boost::json::parse(json_text, ec);
    if (ec || !value.is_object())
      continue;

    const auto &obj = value.as_object();
    const auto it_aircraft = obj.find("aircraft");
    if (it_aircraft == obj.end() || !it_aircraft->value().is_array())
      continue;

    const auto &aircraft = it_aircraft->value().as_array();
    std::size_t ia = 0, ib = 0;
    if (!SelectDistinctAircraftPair(aircraft, cfg, ia, ib))
      continue;

    const auto &a0 = aircraft[ia].as_object();
    const auto &a1 = aircraft[ib].as_object();
    const auto it_t0 = a0.find("trace");
    const auto it_t1 = a1.find("trace");
    if (it_t0 == a0.end() || it_t1 == a1.end() ||
        !it_t0->value().is_array() || !it_t1->value().is_array())
      continue;

    const auto &t0 = it_t0->value().as_array();
    const auto &t1 = it_t1->value().as_array();
    const std::size_t n = std::min<std::size_t>(t0.size(), t1.size());
    if (n < 2)
      continue;

    for (std::size_t k = 1; k < n; ++k)
    {
      if (!t0[k - 1].is_object() || !t1[k - 1].is_object() || !t0[k].is_object() || !t1[k].is_object())
        continue;

      const auto &pa = t0[k - 1].as_object();
      const auto &pb = t1[k - 1].as_object();
      const auto &ca = t0[k].as_object();
      const auto &cb = t1[k].as_object();

      const auto req = [](const boost::json::object &o, const char *key)
      { return o.find(key) != o.end() && o.at(key).is_number(); };
      const auto get_bool = [](const boost::json::object &o, const char *key, bool default_value)
      {
        const auto it = o.find(key);
        if (it == o.end() || !it->value().is_bool())
          return default_value;
        return it->value().as_bool();
      };

      if (!cfg.include_implausible)
      {
        const bool plausible = get_bool(pa, "plausible", true) && get_bool(pb, "plausible", true) &&
                               get_bool(ca, "plausible", true) && get_bool(cb, "plausible", true);
        if (!plausible)
          continue;
      }

      if (!(req(pa, "t") && req(ca, "t") && req(cb, "t")))
        continue;

      const double tpa = pa.at("t").to_number<double>();
      const double tca = ca.at("t").to_number<double>();
      const double tcb = cb.at("t").to_number<double>();
      const double dt = (tca - tpa);
      if (dt < cfg.min_dt_seconds)
        continue;
      if (std::abs(tca - tcb) > cfg.max_time_delta_seconds)
        continue;
      if (cfg.closing_only_until_t0 && tca > cfg.closing_t0_seconds)
        continue;

      const char *alt_key = cfg.use_baro_altitude ? "alt_baro" : "alt_gps";
      if (!(req(pa, "x") && req(pa, "y") && req(pa, alt_key) &&
            req(pb, "x") && req(pb, "y") && req(pb, alt_key) &&
            req(ca, "x") && req(ca, "y") && req(ca, alt_key) &&
            req(cb, "x") && req(cb, "y") && req(cb, alt_key)))
        continue;

      const double pax = pa.at("x").to_number<double>();
      const double pay = pa.at("y").to_number<double>();
      const double pbx = pb.at("x").to_number<double>();
      const double pby = pb.at("y").to_number<double>();
      const double cax = ca.at("x").to_number<double>();
      const double cay = ca.at("y").to_number<double>();
      const double cbx = cb.at("x").to_number<double>();
      const double cby = cb.at("y").to_number<double>();

      const double d0 = std::hypot(pbx - pax, pby - pay);
      const double d1 = std::hypot(cbx - cax, cby - cay);

      const double inv_dt = cfg.normalize_rates ? (1.0 / std::max(dt, 1e-6)) : 1.0;
      dr_samples.push_back((d1 - d0) * inv_dt);

      const double pa_alt = pa.at(alt_key).to_number<double>();
      const double pb_alt = pb.at(alt_key).to_number<double>();
      const double ca_alt = ca.at(alt_key).to_number<double>();
      const double cb_alt = cb.at(alt_key).to_number<double>();
      dz_samples.push_back(((cb_alt - ca_alt) - (pb_alt - pa_alt)) * inv_dt);

      const double ha = std::atan2(cax - pax, cay - pay);
      const double rel0 = WrapAngleRad(std::atan2(pbx - pax, pby - pay) - ha);
      const double rel1 = WrapAngleRad(std::atan2(cbx - cax, cby - cay) - ha);
      dl_samples.push_back(WrapAngleRad(rel1 - rel0) * inv_dt);
    }

    out.encounters_used++;

    if (out.encounters_used >= min_encounters && ((out.encounters_used % check_interval) == 0))
    {
      const double sr = RobustSigmaFromMAD(dr_samples);
      const double sz = RobustSigmaFromMAD(dz_samples);
      const double sl = RobustSigmaFromMAD(dl_samples);

      if (std::isfinite(prev_r) && std::isfinite(prev_z) && std::isfinite(prev_l))
      {
        const auto rel = [](const double a, const double b)
        {
          return std::abs(a - b) / std::max(std::abs(b), 1e-9);
        };

        if (rel(sr, prev_r) < cfg.mad_fit_rel_tolerance &&
            rel(sz, prev_z) < cfg.mad_fit_rel_tolerance &&
            rel(sl, prev_l) < cfg.mad_fit_rel_tolerance)
          stable_rounds++;
        else
          stable_rounds = 0;

        if (stable_rounds >= patience)
          break;
      }

      prev_r = sr;
      prev_z = sz;
      prev_l = sl;
    }
  }

  out.samples = std::min({dr_samples.size(), dz_samples.size(), dl_samples.size()});
  if (out.samples == 0)
    return out;

  const double scale = std::max(0.1, cfg.mad_fit_scale);
  out.radial_zero_m = std::max(std::max(1e-6, cfg.mad_fit_min_radial_zero_m),
                               scale * RobustSigmaFromMAD(dr_samples));
  out.vertical_zero_m = std::max(std::max(1e-6, cfg.mad_fit_min_vertical_zero_m),
                                 scale * RobustSigmaFromMAD(dz_samples));
  out.lateral_zero_rad = std::max(std::max(1e-9, cfg.mad_fit_min_lateral_zero_rad),
                                  scale * RobustSigmaFromMAD(dl_samples));
  out.fitted = true;
  return out;
}

static void
ApplyCovarianceOverrides(const boost::json::object &j,
                         const char *key,
                         bool (*setter)(std::string_view, double))
{
  const auto it = j.find(key);
  if (it == j.end() || !it->value().is_object())
    return;

  const auto &overrides = it->value().as_object();
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

static void DecodeOptions(const boost::json::value &_j,
                          MultiAircraft::FlightCollectionEncounter &flights,
                          VignetteConfig &vignette,
                          AirspaceConfig &airspace,
                          MultiAircraft::CovarianceTuningConfig &covariance_tuning,
                          MultiAircraft::AirfieldList &airfield_list,
                          TrajectoryTokenizerConfig &trajectory_tokenizer)
{
  const auto &j = _j.as_object();
  auto try_apply = [&](const char *key, auto &&fn)
  {
    const auto it = j.find(key);
    if (it == j.end())
      return false;

    try
    {
      fn(it->value());
      return true;
    }
    catch (const boost::system::system_error &)
    {
      return false;
    }
  };

  auto try_apply_report = [&](const char *key, auto &&fn)
  {
    const auto it = j.find(key);
    if (it == j.end())
      return false;

    try
    {
      fn(it->value());
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

  try_apply_report("terrain_clearance", [&](const boost::json::value &v)
                   { flights.TERRAIN_CLEARANCE_M = v.to_number<double>(); });

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
              airspace.openaip_path = std::string(obj.at("openaip").as_string());
              if (auto it = obj.find("incursion_threshold"); it != obj.end())
                airspace.incursion_threshold = it->value().to_number<double>(); });

  try_apply("terrain", [&](const boost::json::value &v)
            {
              const auto &obj = v.as_object();
              const auto &files_array = obj.at("files").as_array();
              std::vector<std::string> files;
              files.reserve(files_array.size());
              for (const auto &entry : files_array)
                files.emplace_back(entry.as_string());

              flights.LoadTerrain(files); });

  try_apply("airfields", [&](const boost::json::value &v)
            {
              for (const auto &entry : v.as_array())
              {
                const auto &obj = entry.as_object();
                const double lat = obj.at("latitude").to_number<double>();
                const double lon = obj.at("longitude").to_number<double>();
                std::string name;
                if (auto it = obj.find("name"); it != obj.end())
                  name = std::string(it->value().as_string());
                airfield_list.Add(
                    GeoPoint(Angle::Degrees(lon), Angle::Degrees(lat)),
                    std::move(name));
              } });

  try_apply("covariance_tuning", [&](const boost::json::value &v)
            { DecodeCovarianceTuning(v.as_object(), covariance_tuning); });

  try_apply("trajectory_tokenizer", [&](const boost::json::value &v)
            {
              const auto &obj = v.as_object();
              trajectory_tokenizer.enabled = true;

              if (auto it = obj.find("enabled"); it != obj.end())
                trajectory_tokenizer.enabled = it->value().as_bool();
              if (auto it = obj.find("subject"); it != obj.end())
                trajectory_tokenizer.subject = std::string(it->value().as_string());
              if (auto it = obj.find("output"); it != obj.end())
                trajectory_tokenizer.output = std::string(it->value().as_string());
              if (auto it = obj.find("max_time_delta_seconds"); it != obj.end())
                trajectory_tokenizer.max_time_delta_seconds = it->value().to_number<double>();
              if (auto it = obj.find("min_dt_seconds"); it != obj.end())
                trajectory_tokenizer.min_dt_seconds = it->value().to_number<double>();
              if (auto it = obj.find("use_baro_altitude"); it != obj.end())
                trajectory_tokenizer.use_baro_altitude = it->value().as_bool();
              if (auto it = obj.find("include_implausible"); it != obj.end())
                trajectory_tokenizer.include_implausible = it->value().as_bool();
              if (auto it = obj.find("radial_zero_m"); it != obj.end())
                trajectory_tokenizer.radial_zero_m = it->value().to_number<double>();
              if (auto it = obj.find("vertical_zero_m"); it != obj.end())
                trajectory_tokenizer.vertical_zero_m = it->value().to_number<double>();
              if (auto it = obj.find("lateral_zero_rad"); it != obj.end())
                trajectory_tokenizer.lateral_zero_rad = it->value().to_number<double>();

              if (auto it = obj.find("use_custom_levels"); it != obj.end())
                trajectory_tokenizer.use_custom_levels = it->value().as_bool();
              if (auto it = obj.find("normalize_rates"); it != obj.end())
                trajectory_tokenizer.normalize_rates = it->value().as_bool();
              if (auto it = obj.find("nonuniform_bins"); it != obj.end())
                trajectory_tokenizer.nonuniform_bins = it->value().as_bool();
              if (auto it = obj.find("nonuniform_bin_exponent"); it != obj.end())
                trajectory_tokenizer.nonuniform_bin_exponent = it->value().to_number<double>();
              if (auto it = obj.find("closing_only_until_t0"); it != obj.end())
                trajectory_tokenizer.closing_only_until_t0 = it->value().as_bool();
              if (auto it = obj.find("closing_t0_seconds"); it != obj.end())
                trajectory_tokenizer.closing_t0_seconds = it->value().to_number<double>();
              if (auto it = obj.find("radial_levels"); it != obj.end())
                trajectory_tokenizer.radial_levels = it->value().to_number<unsigned>();
              if (auto it = obj.find("vertical_levels"); it != obj.end())
                trajectory_tokenizer.vertical_levels = it->value().to_number<unsigned>();
              if (auto it = obj.find("lateral_levels"); it != obj.end())
                trajectory_tokenizer.lateral_levels = it->value().to_number<unsigned>();
              if (auto it = obj.find("bank_levels"); it != obj.end())
                trajectory_tokenizer.bank_levels = it->value().to_number<unsigned>();
              if (auto it = obj.find("dbank_levels"); it != obj.end())
                trajectory_tokenizer.dbank_levels = it->value().to_number<unsigned>();

              if (auto it = obj.find("radial_min"); it != obj.end())
                trajectory_tokenizer.radial_min = it->value().to_number<double>();
              if (auto it = obj.find("radial_max"); it != obj.end())
                trajectory_tokenizer.radial_max = it->value().to_number<double>();
              if (auto it = obj.find("vertical_min"); it != obj.end())
                trajectory_tokenizer.vertical_min = it->value().to_number<double>();
              if (auto it = obj.find("vertical_max"); it != obj.end())
                trajectory_tokenizer.vertical_max = it->value().to_number<double>();
              if (auto it = obj.find("lateral_min"); it != obj.end())
                trajectory_tokenizer.lateral_min = it->value().to_number<double>();
              if (auto it = obj.find("lateral_max"); it != obj.end())
                trajectory_tokenizer.lateral_max = it->value().to_number<double>();
              if (auto it = obj.find("bank_min_deg"); it != obj.end())
                trajectory_tokenizer.bank_min_deg = it->value().to_number<double>();
              if (auto it = obj.find("bank_max_deg"); it != obj.end())
                trajectory_tokenizer.bank_max_deg = it->value().to_number<double>();
              if (auto it = obj.find("dbank_min_deg"); it != obj.end())
                trajectory_tokenizer.dbank_min_deg = it->value().to_number<double>();
              if (auto it = obj.find("dbank_max_deg"); it != obj.end())
                trajectory_tokenizer.dbank_max_deg = it->value().to_number<double>();

              if (auto it = obj.find("fit_mad_thresholds"); it != obj.end())
                trajectory_tokenizer.fit_mad_thresholds = it->value().as_bool();
              if (auto it = obj.find("mad_fit_scale"); it != obj.end())
                trajectory_tokenizer.mad_fit_scale = it->value().to_number<double>();
              if (auto it = obj.find("mad_fit_min_encounters"); it != obj.end())
                trajectory_tokenizer.mad_fit_min_encounters = it->value().to_number<unsigned>();
              if (auto it = obj.find("mad_fit_max_encounters"); it != obj.end())
                trajectory_tokenizer.mad_fit_max_encounters = it->value().to_number<unsigned>();
              if (auto it = obj.find("mad_fit_check_interval"); it != obj.end())
                trajectory_tokenizer.mad_fit_check_interval = it->value().to_number<unsigned>();
              if (auto it = obj.find("mad_fit_rel_tolerance"); it != obj.end())
                trajectory_tokenizer.mad_fit_rel_tolerance = it->value().to_number<double>();
              if (auto it = obj.find("mad_fit_patience"); it != obj.end())
                trajectory_tokenizer.mad_fit_patience = it->value().to_number<unsigned>();
              if (auto it = obj.find("mad_fit_min_radial_zero_m"); it != obj.end())
                trajectory_tokenizer.mad_fit_min_radial_zero_m = it->value().to_number<double>();
              if (auto it = obj.find("mad_fit_min_vertical_zero_m"); it != obj.end())
                trajectory_tokenizer.mad_fit_min_vertical_zero_m = it->value().to_number<double>();
              if (auto it = obj.find("mad_fit_min_lateral_zero_rad"); it != obj.end())
                trajectory_tokenizer.mad_fit_min_lateral_zero_rad = it->value().to_number<double>();
              if (auto it = obj.find("include_bank_state"); it != obj.end())
                trajectory_tokenizer.include_bank_state = it->value().as_bool();
              if (auto it = obj.find("bank_zero_deg"); it != obj.end())
                trajectory_tokenizer.bank_zero_deg = it->value().to_number<double>();
              if (auto it = obj.find("include_bank_trend"); it != obj.end())
                trajectory_tokenizer.include_bank_trend = it->value().as_bool();
              if (auto it = obj.find("bank_delta_zero_deg"); it != obj.end())
                trajectory_tokenizer.bank_delta_zero_deg = it->value().to_number<double>();
              if (auto it = obj.find("canonicalize_pair_order"); it != obj.end())
                trajectory_tokenizer.canonicalize_pair_order = it->value().as_bool();
              if (auto it = obj.find("handedness_invariant"); it != obj.end())
                trajectory_tokenizer.handedness_invariant = it->value().as_bool();
              if (auto it = obj.find("reject_near_identical_pair"); it != obj.end())
                trajectory_tokenizer.reject_near_identical_pair = it->value().as_bool();
              if (auto it = obj.find("pair_identical_xy_tol_m"); it != obj.end())
                trajectory_tokenizer.pair_identical_xy_tol_m = it->value().to_number<double>();
              if (auto it = obj.find("pair_identical_alt_tol_m"); it != obj.end())
                trajectory_tokenizer.pair_identical_alt_tol_m = it->value().to_number<double>();
              if (auto it = obj.find("pair_identical_min_fraction"); it != obj.end())
                trajectory_tokenizer.pair_identical_min_fraction = it->value().to_number<double>();
              if (auto it = obj.find("rle_encode"); it != obj.end())
                trajectory_tokenizer.rle_encode = it->value().as_bool();
              if (auto it = obj.find("include_raw_tokens"); it != obj.end())
                trajectory_tokenizer.include_raw_tokens = it->value().as_bool();
              if (auto it = obj.find("max_tokens_per_pair"); it != obj.end())
                trajectory_tokenizer.max_tokens_per_pair = it->value().to_number<unsigned>();
            });

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
#ifdef GIT_SHA
  std::cout << "RunMultiAircraft [" << GIT_SHA << "]" << std::endl;
#endif

  MultiAircraft::FlightCollectionEncounter flights;
  VignetteConfig vignette;
  AirspaceConfig airspace;
  MultiAircraft::CovarianceTuningConfig covariance_tuning;
  MultiAircraft::AirfieldList airfield_list;
  TrajectoryTokenizerConfig trajectory_tokenizer;

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

  DecodeOptions(json_data, flights, vignette, airspace, covariance_tuning,
                airfield_list, trajectory_tokenizer);

  if (!airfield_list.empty())
  {
    std::cout << "Loaded " << airfield_list.size() << " airfield(s)\n";
    flights.SetAirfieldList(std::move(airfield_list));
  }
  MultiAircraft::AircraftModel::SetWriteTraceFiles(!vignette.enabled);
  MultiAircraft::AircraftModel::SetKeepFullTrail(vignette.enabled ||
                                                 covariance_tuning.enabled);
  MultiAircraft::FlightCollectionEncounter::SetSkipEncounterProcessing(covariance_tuning.enabled);
  MultiAircraft::FlightFlock::SetWriteJsonFile(!vignette.enabled);
  if (airspace.enabled())
  {
    flights.LoadOpenAipAirspaces(Path(airspace.openaip_path.c_str()));
    flights.INCURSION_THRESHOLD_M = airspace.incursion_threshold;
  }
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

  if (trajectory_tokenizer.enabled)
  {
    MultiAircraft::TrajectoryTokenizerQTC3D::Config qcfg;
    qcfg.max_time_delta_seconds = trajectory_tokenizer.max_time_delta_seconds;
    qcfg.min_dt_seconds = trajectory_tokenizer.min_dt_seconds;
    qcfg.use_baro_altitude = trajectory_tokenizer.use_baro_altitude;
    qcfg.include_implausible = trajectory_tokenizer.include_implausible;
    qcfg.radial_zero_m = trajectory_tokenizer.radial_zero_m;
    qcfg.vertical_zero_m = trajectory_tokenizer.vertical_zero_m;
    qcfg.lateral_zero_rad = trajectory_tokenizer.lateral_zero_rad;
    qcfg.include_bank_state = trajectory_tokenizer.include_bank_state;
    qcfg.bank_zero_deg = trajectory_tokenizer.bank_zero_deg;
    qcfg.include_bank_trend = trajectory_tokenizer.include_bank_trend;
    qcfg.bank_delta_zero_deg = trajectory_tokenizer.bank_delta_zero_deg;

    MADFitThresholds mad_fit;
    if (trajectory_tokenizer.fit_mad_thresholds)
    {
      mad_fit = FitMADThresholdsFromEncounterFiles(trajectory_tokenizer);
      if (mad_fit.fitted)
      {
        qcfg.radial_zero_m = mad_fit.radial_zero_m;
        qcfg.vertical_zero_m = mad_fit.vertical_zero_m;
        qcfg.lateral_zero_rad = mad_fit.lateral_zero_rad;

        std::cout << "MAD threshold fit applied: radial=" << qcfg.radial_zero_m
                  << " vertical=" << qcfg.vertical_zero_m
                  << " lateral=" << qcfg.lateral_zero_rad
                  << " encounters=" << mad_fit.encounters_used
                  << " samples=" << mad_fit.samples << "\n";
      }
      else
      {
        std::cout << "MAD threshold fit skipped (insufficient encounter samples), using configured thresholds\n";
      }
    }

    boost::json::object tokens_root;
    tokens_root["tokenizer"] = "QTC3D";

    boost::json::object tokenizer_cfg;
    tokenizer_cfg["max_time_delta_seconds"] = trajectory_tokenizer.max_time_delta_seconds;
    tokenizer_cfg["min_dt_seconds"] = trajectory_tokenizer.min_dt_seconds;
    tokenizer_cfg["use_baro_altitude"] = trajectory_tokenizer.use_baro_altitude;
    tokenizer_cfg["include_implausible"] = trajectory_tokenizer.include_implausible;
    tokenizer_cfg["fit_mad_thresholds"] = trajectory_tokenizer.fit_mad_thresholds;
    tokenizer_cfg["mad_fit_scale"] = trajectory_tokenizer.mad_fit_scale;
    tokenizer_cfg["mad_fit_min_encounters"] = trajectory_tokenizer.mad_fit_min_encounters;
    tokenizer_cfg["mad_fit_max_encounters"] = trajectory_tokenizer.mad_fit_max_encounters;
    tokenizer_cfg["mad_fit_check_interval"] = trajectory_tokenizer.mad_fit_check_interval;
    tokenizer_cfg["mad_fit_rel_tolerance"] = trajectory_tokenizer.mad_fit_rel_tolerance;
    tokenizer_cfg["mad_fit_patience"] = trajectory_tokenizer.mad_fit_patience;
    tokenizer_cfg["mad_fit_min_radial_zero_m"] = trajectory_tokenizer.mad_fit_min_radial_zero_m;
    tokenizer_cfg["mad_fit_min_vertical_zero_m"] = trajectory_tokenizer.mad_fit_min_vertical_zero_m;
    tokenizer_cfg["mad_fit_min_lateral_zero_rad"] = trajectory_tokenizer.mad_fit_min_lateral_zero_rad;
    tokenizer_cfg["radial_zero_m"] = qcfg.radial_zero_m;
    tokenizer_cfg["vertical_zero_m"] = qcfg.vertical_zero_m;
    tokenizer_cfg["lateral_zero_rad"] = qcfg.lateral_zero_rad;
    tokenizer_cfg["use_custom_levels"] = trajectory_tokenizer.use_custom_levels;
    tokenizer_cfg["normalize_rates"] = trajectory_tokenizer.normalize_rates;
    tokenizer_cfg["nonuniform_bins"] = trajectory_tokenizer.nonuniform_bins;
    tokenizer_cfg["nonuniform_bin_exponent"] = trajectory_tokenizer.nonuniform_bin_exponent;
    tokenizer_cfg["closing_only_until_t0"] = trajectory_tokenizer.closing_only_until_t0;
    tokenizer_cfg["closing_t0_seconds"] = trajectory_tokenizer.closing_t0_seconds;
    tokenizer_cfg["radial_levels"] = trajectory_tokenizer.radial_levels;
    tokenizer_cfg["vertical_levels"] = trajectory_tokenizer.vertical_levels;
    tokenizer_cfg["lateral_levels"] = trajectory_tokenizer.lateral_levels;
    tokenizer_cfg["bank_levels"] = trajectory_tokenizer.bank_levels;
    tokenizer_cfg["dbank_levels"] = trajectory_tokenizer.dbank_levels;
    tokenizer_cfg["radial_min"] = trajectory_tokenizer.radial_min;
    tokenizer_cfg["radial_max"] = trajectory_tokenizer.radial_max;
    tokenizer_cfg["vertical_min"] = trajectory_tokenizer.vertical_min;
    tokenizer_cfg["vertical_max"] = trajectory_tokenizer.vertical_max;
    tokenizer_cfg["lateral_min"] = trajectory_tokenizer.lateral_min;
    tokenizer_cfg["lateral_max"] = trajectory_tokenizer.lateral_max;
    tokenizer_cfg["bank_min_deg"] = trajectory_tokenizer.bank_min_deg;
    tokenizer_cfg["bank_max_deg"] = trajectory_tokenizer.bank_max_deg;
    tokenizer_cfg["dbank_min_deg"] = trajectory_tokenizer.dbank_min_deg;
    tokenizer_cfg["dbank_max_deg"] = trajectory_tokenizer.dbank_max_deg;
    tokenizer_cfg["mad_fit_applied"] = mad_fit.fitted;
    tokenizer_cfg["mad_fit_encounters_used"] = static_cast<std::uint64_t>(mad_fit.encounters_used);
    tokenizer_cfg["mad_fit_samples"] = static_cast<std::uint64_t>(mad_fit.samples);
    tokenizer_cfg["include_bank_state"] = trajectory_tokenizer.include_bank_state;
    tokenizer_cfg["bank_zero_deg"] = trajectory_tokenizer.bank_zero_deg;
    tokenizer_cfg["include_bank_trend"] = trajectory_tokenizer.include_bank_trend;
    tokenizer_cfg["bank_delta_zero_deg"] = trajectory_tokenizer.bank_delta_zero_deg;
    tokenizer_cfg["canonicalize_pair_order"] = trajectory_tokenizer.canonicalize_pair_order;
    tokenizer_cfg["handedness_invariant"] = trajectory_tokenizer.handedness_invariant;
    tokenizer_cfg["reject_near_identical_pair"] = trajectory_tokenizer.reject_near_identical_pair;
    tokenizer_cfg["pair_identical_xy_tol_m"] = trajectory_tokenizer.pair_identical_xy_tol_m;
    tokenizer_cfg["pair_identical_alt_tol_m"] = trajectory_tokenizer.pair_identical_alt_tol_m;
    tokenizer_cfg["pair_identical_min_fraction"] = trajectory_tokenizer.pair_identical_min_fraction;
    tokenizer_cfg["rle_encode"] = trajectory_tokenizer.rle_encode;
    tokenizer_cfg["include_raw_tokens"] = trajectory_tokenizer.include_raw_tokens;
    tokenizer_cfg["max_tokens_per_pair"] = trajectory_tokenizer.max_tokens_per_pair;
    tokens_root["tokenizer_config"] = std::move(tokenizer_cfg);

    auto get_number = [](const boost::json::object &o, const char *key, double &out_value)
    {
      const auto it = o.find(key);
      if (it == o.end() || !it->value().is_number())
        return false;
      out_value = it->value().to_number<double>();
      return true;
    };

    auto get_bool = [](const boost::json::object &o, const char *key, bool default_value)
    {
      const auto it = o.find(key);
      if (it == o.end() || !it->value().is_bool())
        return default_value;
      return it->value().as_bool();
    };

    auto sign_token = [](double value, double eps)
    {
      if (value > eps)
        return std::string("+");
      if (value < -eps)
        return std::string("-");
      return std::string("0");
    };

    auto normalize_levels = [](unsigned levels)
    {
      levels = std::max(3u, levels);
      if ((levels % 2u) == 0u)
        ++levels;
      return levels;
    };

    auto quantize_level = [&](const double value,
                              const double min_value,
                              const double max_value,
                              const unsigned levels)
    {
      const auto L = normalize_levels(levels);
      if (!(max_value > min_value))
        return std::string("0");

      const double clamped = std::clamp(value, min_value, max_value);
      const double pos = (clamped - min_value) / (max_value - min_value);
      int idx = static_cast<int>(std::llround(pos * static_cast<double>(L - 1)));
      idx = std::clamp(idx, 0, static_cast<int>(L - 1));
      const int center = static_cast<int>((L - 1) / 2);
      const int signed_level = idx - center;
      return std::to_string(signed_level);
    };

    auto quantize_level_nonuniform = [&](const double value,
                                         const double zero_eps,
                                         const double min_value,
                                         const double max_value,
                                         const unsigned levels)
    {
      const auto L = normalize_levels(levels);
      const int half = static_cast<int>((L - 1) / 2);
      if (half <= 0)
        return std::string("0");

      const double abs_value = std::abs(value);
      if (abs_value <= zero_eps)
        return std::string("0");

      const double amax = std::max(std::abs(min_value), std::abs(max_value));
      if (!(amax > zero_eps + 1e-9))
        return sign_token(value, zero_eps);

      const double t = std::clamp((abs_value - zero_eps) / (amax - zero_eps), 0.0, 1.0);
      const double exponent = std::clamp(trajectory_tokenizer.nonuniform_bin_exponent, 0.05, 10.0);
      const double warped = std::pow(t, exponent);
      int mag = 1 + static_cast<int>(std::floor(warped * static_cast<double>(half)));
      mag = std::clamp(mag, 1, half);

      const int signed_level = value < 0 ? -mag : mag;
      return std::to_string(signed_level);
    };

    auto encode_axis = [&](double value,
                           double zero_eps,
                           double min_value,
                           double max_value,
                           unsigned levels)
    {
      if (!trajectory_tokenizer.use_custom_levels)
        return sign_token(value, zero_eps);
      if (trajectory_tokenizer.nonuniform_bins)
        return quantize_level_nonuniform(value, zero_eps, min_value, max_value, levels);
      return quantize_level(value, min_value, max_value, levels);
    };

    std::vector<std::filesystem::path> encounter_files;
    for (const auto &entry : std::filesystem::directory_iterator(std::filesystem::current_path()))
    {
      if (!entry.is_regular_file())
        continue;

      const auto name = entry.path().filename().string();
      if (name.rfind("encounter_", 0) == 0 && entry.path().extension() == ".json")
        encounter_files.push_back(entry.path());
    }
    std::sort(encounter_files.begin(), encounter_files.end());

    boost::json::array encounters_json;

    for (const auto &path : encounter_files)
    {
      std::ifstream in(path);
      if (!in)
        continue;

      const std::string json_text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
      boost::system::error_code ec;
      const auto parsed = boost::json::parse(json_text, ec);
      if (ec || !parsed.is_object())
        continue;

      const auto &obj = parsed.as_object();
      const auto it_aircraft = obj.find("aircraft");
      if (it_aircraft == obj.end() || !it_aircraft->value().is_array())
        continue;

      const auto &aircraft_array = it_aircraft->value().as_array();
      std::size_t ia = 0, ib = 0;
      if (!SelectDistinctAircraftPair(aircraft_array, trajectory_tokenizer, ia, ib))
        continue;

      const auto &a_obj = aircraft_array[ia].as_object();
      const auto &b_obj = aircraft_array[ib].as_object();

      std::string a_id = a_obj.contains("id") ? std::string(a_obj.at("id").as_string()) : std::string();
      std::string b_id = b_obj.contains("id") ? std::string(b_obj.at("id").as_string()) : std::string();
      std::string a_fr = a_obj.contains("fr_id") ? std::string(a_obj.at("fr_id").as_string()) : std::string();
      std::string b_fr = b_obj.contains("fr_id") ? std::string(b_obj.at("fr_id").as_string()) : std::string();

      if (!trajectory_tokenizer.subject.empty() &&
          a_id != trajectory_tokenizer.subject && a_fr != trajectory_tokenizer.subject &&
          b_id != trajectory_tokenizer.subject && b_fr != trajectory_tokenizer.subject)
        continue;

      const auto it_ta = a_obj.find("trace");
      const auto it_tb = b_obj.find("trace");
      if (it_ta == a_obj.end() || it_tb == b_obj.end() ||
          !it_ta->value().is_array() || !it_tb->value().is_array())
        continue;

      const auto &ta = it_ta->value().as_array();
      const auto &tb = it_tb->value().as_array();
      const std::size_t n = std::min<std::size_t>(ta.size(), tb.size());
      if (n < 2)
        continue;

      std::vector<std::string> tokens;
      tokens.reserve(n - 1);
      const char *alt_key = qcfg.use_baro_altitude ? "alt_baro" : "alt_gps";

      for (std::size_t k = 1; k < n; ++k)
      {
        if (!ta[k - 1].is_object() || !tb[k - 1].is_object() || !ta[k].is_object() || !tb[k].is_object())
          continue;

        const auto &pa = ta[k - 1].as_object();
        const auto &pb = tb[k - 1].as_object();
        const auto &ca = ta[k].as_object();
        const auto &cb = tb[k].as_object();

        if (!trajectory_tokenizer.include_implausible)
        {
          const bool plausible = get_bool(pa, "plausible", true) && get_bool(pb, "plausible", true) &&
                                 get_bool(ca, "plausible", true) && get_bool(cb, "plausible", true);
          if (!plausible)
            continue;
        }

        double tpa = 0, tca = 0, tcb = 0;
        if (!(get_number(pa, "t", tpa) && get_number(ca, "t", tca) && get_number(cb, "t", tcb)))
          continue;

        const double dt = (tca - tpa);
        if (dt < trajectory_tokenizer.min_dt_seconds)
          continue;
        if (std::abs(tca - tcb) > trajectory_tokenizer.max_time_delta_seconds)
          continue;
        if (trajectory_tokenizer.closing_only_until_t0 && tca > trajectory_tokenizer.closing_t0_seconds)
          continue;

        double pax, pay, pa_alt, pbx, pby, pb_alt, cax, cay, ca_alt, cbx, cby, cb_alt;
        if (!(get_number(pa, "x", pax) && get_number(pa, "y", pay) && get_number(pa, alt_key, pa_alt) &&
              get_number(pb, "x", pbx) && get_number(pb, "y", pby) && get_number(pb, alt_key, pb_alt) &&
              get_number(ca, "x", cax) && get_number(ca, "y", cay) && get_number(ca, alt_key, ca_alt) &&
              get_number(cb, "x", cbx) && get_number(cb, "y", cby) && get_number(cb, alt_key, cb_alt)))
          continue;

        const double d0 = std::hypot(pbx - pax, pby - pay);
        const double d1 = std::hypot(cbx - cax, cby - cay);
        const double inv_dt = trajectory_tokenizer.normalize_rates ? (1.0 / std::max(dt, 1e-6)) : 1.0;

        const double radial_value = (d1 - d0) * inv_dt;
        const std::string r = encode_axis(radial_value,
                                          qcfg.radial_zero_m,
                                          trajectory_tokenizer.radial_min,
                                          trajectory_tokenizer.radial_max,
                                          trajectory_tokenizer.radial_levels);

        const double vertical_value = ((cb_alt - ca_alt) - (pb_alt - pa_alt)) * inv_dt;
        const std::string z = encode_axis(vertical_value,
                                          qcfg.vertical_zero_m,
                                          trajectory_tokenizer.vertical_min,
                                          trajectory_tokenizer.vertical_max,
                                          trajectory_tokenizer.vertical_levels);

        const double ha = std::atan2(cax - pax, cay - pay);
        const double rel0 = WrapAngleRad(std::atan2(pbx - pax, pby - pay) - ha);
        const double rel1 = WrapAngleRad(std::atan2(cbx - cax, cby - cay) - ha);
        const double lateral_value = WrapAngleRad(rel1 - rel0) * inv_dt;
        const std::string l = encode_axis(lateral_value,
                                          qcfg.lateral_zero_rad,
                                          trajectory_tokenizer.lateral_min,
                                          trajectory_tokenizer.lateral_max,
                                          trajectory_tokenizer.lateral_levels);

        std::string token = std::string("QTC3D(r=") + r + ",z=" + z + ",l=" + l + ")";

        if (trajectory_tokenizer.include_bank_state)
        {
          double ba = 0, bb = 0;
          (void)get_number(ca, "bank", ba);
          (void)get_number(cb, "bank", bb);
          token += std::string("|BANK(bA=") + encode_axis(ba,
                                                           trajectory_tokenizer.bank_zero_deg,
                                                           trajectory_tokenizer.bank_min_deg,
                                                           trajectory_tokenizer.bank_max_deg,
                                                           trajectory_tokenizer.bank_levels) +
                   ",bB=" + encode_axis(bb,
                                         trajectory_tokenizer.bank_zero_deg,
                                         trajectory_tokenizer.bank_min_deg,
                                         trajectory_tokenizer.bank_max_deg,
                                         trajectory_tokenizer.bank_levels) + ")";
        }

        if (trajectory_tokenizer.include_bank_trend)
        {
          double pba = 0, pbb = 0, cba = 0, cbb = 0;
          (void)get_number(pa, "bank", pba);
          (void)get_number(pb, "bank", pbb);
          (void)get_number(ca, "bank", cba);
          (void)get_number(cb, "bank", cbb);
          token += std::string("|DBANK(dbA=") + encode_axis(cba - pba,
                                                             trajectory_tokenizer.bank_delta_zero_deg,
                                                             trajectory_tokenizer.dbank_min_deg,
                                                             trajectory_tokenizer.dbank_max_deg,
                                                             trajectory_tokenizer.dbank_levels) +
                   ",dbB=" + encode_axis(cbb - pbb,
                                          trajectory_tokenizer.bank_delta_zero_deg,
                                          trajectory_tokenizer.dbank_min_deg,
                                          trajectory_tokenizer.dbank_max_deg,
                                          trajectory_tokenizer.dbank_levels) + ")";
        }

        tokens.push_back(std::move(token));
      }

      bool swapped_for_canonical = false;
      if (trajectory_tokenizer.canonicalize_pair_order)
      {
        const std::string a_key = !a_id.empty() ? a_id : a_fr;
        const std::string b_key = !b_id.empty() ? b_id : b_fr;
        if (b_key < a_key)
        {
          swapped_for_canonical = true;
          for (auto &t : tokens)
            t = MultiAircraft::CanonicalizeQTC3DTokenForSwappedPair(t);

          std::swap(a_id, b_id);
          std::swap(a_fr, b_fr);
        }
      }

      if (trajectory_tokenizer.handedness_invariant)
      {
        for (auto &t : tokens)
        {
          const auto mirrored = MultiAircraft::MirrorQTC3DTokenHandedness(t);
          if (mirrored < t)
            t = mirrored;
        }
      }

      if (trajectory_tokenizer.max_tokens_per_pair > 0 &&
          tokens.size() > trajectory_tokenizer.max_tokens_per_pair)
      {
        tokens.resize(trajectory_tokenizer.max_tokens_per_pair);
      }

      boost::json::object enc_obj;
      enc_obj["encounter_file"] = path.filename().string();
      if (obj.contains("time_start"))
        enc_obj["time_start"] = obj.at("time_start");
      if (obj.contains("time_end"))
        enc_obj["time_end"] = obj.at("time_end");
      if (obj.contains("d_min"))
        enc_obj["d_min"] = obj.at("d_min");

      enc_obj["a_id"] = a_id;
      enc_obj["a_fr_id"] = a_fr;
      enc_obj["b_id"] = b_id;
      enc_obj["b_fr_id"] = b_fr;
      enc_obj["token_count"] = tokens.size();
      enc_obj["canonicalized_swap"] = swapped_for_canonical;

      if (trajectory_tokenizer.include_raw_tokens)
      {
        boost::json::array token_array;
        for (const auto &t : tokens)
          token_array.push_back(boost::json::value(t));
        enc_obj["tokens"] = std::move(token_array);
      }

      if (trajectory_tokenizer.rle_encode)
      {
        boost::json::array rle_array;
        if (!tokens.empty())
        {
          std::string current = tokens.front();
          std::size_t count = 1;
          for (std::size_t ti = 1; ti < tokens.size(); ++ti)
          {
            if (tokens[ti] == current)
            {
              ++count;
            }
            else
            {
              boost::json::object run;
              run["token"] = current;
              run["count"] = static_cast<std::uint64_t>(count);
              rle_array.push_back(std::move(run));
              current = tokens[ti];
              count = 1;
            }
          }

          boost::json::object run;
          run["token"] = current;
          run["count"] = static_cast<std::uint64_t>(count);
          rle_array.push_back(std::move(run));
        }

        enc_obj["tokens_rle"] = std::move(rle_array);
        enc_obj["token_count_rle"] = enc_obj.at("tokens_rle").as_array().size();
      }

      encounters_json.push_back(std::move(enc_obj));
    }

    tokens_root["encounters"] = std::move(encounters_json);

    std::ofstream token_file(trajectory_tokenizer.output);
    token_file << boost::json::serialize(tokens_root) << "\n";

    std::cout << "Wrote QTC3D tokens to " << trajectory_tokenizer.output << "\n";
  }

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
