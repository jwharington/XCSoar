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
        double blend = 0.5;
        double min_diagonal = 1e-4;
        std::size_t max_flights = 0;
        std::size_t max_points_per_flight = 1500;
        std::size_t max_failures_per_flight = 1500;
        std::size_t max_consecutive_failures = 250;
        std::size_t max_restarts_per_flight = 12;
        std::size_t bootstrap_iterations = 200;
        double bootstrap_fraction = 0.35;
        unsigned random_seed = 1337;
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
