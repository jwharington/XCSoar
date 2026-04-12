// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "AircraftModel.hpp"

#include <boost/json.hpp>

#include <list>
#include <string>

namespace MultiAircraft
{

    struct CovarianceTuningConfig
    {
        bool enabled = false;
        std::string output_path = "ukf_covariance_tuned.json";
        std::size_t min_samples = 50;
        double blend = 0.7;
        double min_diagonal = 1e-4;
        std::size_t max_flights = 48;
        std::size_t max_points_per_flight = 200;
        std::size_t max_failures_per_flight = 120;
        std::size_t max_consecutive_failures = 30;
        std::size_t max_restarts_per_flight = 4;
        std::size_t bootstrap_iterations = 32;
        double bootstrap_fraction = 0.5;
        unsigned random_seed = 1337;

        bool convergence_enabled = true;
        std::size_t convergence_min_passes = 4;
        std::size_t convergence_max_passes = 10;
        std::size_t convergence_flights_per_pass = 48;
        double convergence_rel_tolerance = 0.02;
    };

    struct CovarianceTuningResult
    {
        bool success = false;
        std::size_t measurement_samples = 0;
        std::size_t process_samples = 0;
        std::size_t initial_state_samples = 0;
        boost::json::object covariances;
        std::string message;
    };

    CovarianceTuningResult TuneFlightReconstructionCovariances(
        const std::list<AircraftModel> &aircraft,
        const CovarianceTuningConfig &config);

} // namespace MultiAircraft
