// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "CovarianceTuner.hpp"

#include "FlightReconstruction.hpp"
#include "FlightReconstructionOptions.hpp"
#include "ReconstructionUtility.hpp"
#include "Geo/Flat/FlatProjection.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

namespace MultiAircraft
{
    namespace
    {
        using ProcessDiag = std::array<double, 9>;
        using MeasurementDiag = std::array<double, 4>;
        using StateDiag = std::array<double, 9>;

        double ComputeVariance(const std::vector<double> &values)
        {
            if (values.size() < 2)
                return 0.0;

            const double inv_n = 1.0 / static_cast<double>(values.size());
            double sum = 0.0;
            double sum_sq = 0.0;
            for (const double v : values)
            {
                sum += v;
                sum_sq += v * v;
            }

            const double mean = sum * inv_n;
            const double variance = sum_sq * inv_n - mean * mean;
            return variance > 0.0 ? variance : 0.0;
        }

        double BootstrapVarianceEstimate(const std::vector<double> &values,
                                         const CovarianceTuningConfig &config,
                                         std::mt19937 &rng,
                                         const char *label)
        {
            const double empirical = ComputeVariance(values);
            if (values.size() < 4 || config.bootstrap_iterations == 0)
                return empirical;

            const double fraction = std::clamp(config.bootstrap_fraction, 0.05, 1.0);
            std::size_t sample_size = static_cast<std::size_t>(
                std::round(fraction * static_cast<double>(values.size())));
            sample_size = std::max<std::size_t>(4, std::min(sample_size, values.size()));

            std::uniform_int_distribution<std::size_t> draw(0, values.size() - 1);
            std::vector<double> bootstrap_sample(sample_size);
            std::vector<double> boot_vars;
            boot_vars.reserve(config.bootstrap_iterations);

            std::cout << "[cov-tune] bootstrap begin label=" << label
                      << " n_values=" << values.size()
                      << " sample_size=" << sample_size
                      << " iters=" << config.bootstrap_iterations << "\n";

            const std::size_t log_stride = std::max<std::size_t>(
                static_cast<std::size_t>(1), config.bootstrap_iterations / 4);

            for (std::size_t i = 0; i < config.bootstrap_iterations; ++i)
            {
                for (std::size_t j = 0; j < sample_size; ++j)
                    bootstrap_sample[j] = values[draw(rng)];
                boot_vars.push_back(ComputeVariance(bootstrap_sample));

                if ((i + 1) % log_stride == 0 || (i + 1) == config.bootstrap_iterations)
                {
                    std::cout << "[cov-tune] bootstrap progress label=" << label
                              << " " << (i + 1) << "/" << config.bootstrap_iterations << "\n";
                }
            }

            auto mid = boot_vars.begin() + static_cast<std::ptrdiff_t>(boot_vars.size() / 2);
            std::nth_element(boot_vars.begin(), mid, boot_vars.end());
            const double robust = *mid;

            std::cout << "[cov-tune] bootstrap done label=" << label
                      << " empirical_var=" << empirical
                      << " robust_var=" << robust << "\n";

            return 0.5 * empirical + 0.5 * robust;
        }

        template <std::size_t N>
        using ResidualPool = std::array<std::vector<double>, N>;

        template <std::size_t N>
        std::array<double, N> BuildDiagonal(const ResidualPool<N> &pool,
                                            const std::array<double, N> &baseline,
                                            const CovarianceTuningConfig &config,
                                            std::mt19937 &rng,
                                            const std::array<const char *, N> &labels)
        {
            std::array<double, N> tuned{};
            const double blend = std::clamp(config.blend, 0.0, 1.0);

            for (std::size_t i = 0; i < N; ++i)
            {
                const double robust = BootstrapVarianceEstimate(pool[i], config, rng, labels[i]);
                const double blended = baseline[i] * (1.0 - blend) + robust * blend;
                tuned[i] = std::max(config.min_diagonal, blended);
            }

            return tuned;
        }

        template <std::size_t N>
        void ConstrainEqualPair(std::array<double, N> &diag,
                                const std::size_t i,
                                const std::size_t j)
        {
            const double v = 0.5 * (diag[i] + diag[j]);
            diag[i] = v;
            diag[j] = v;
        }

        template <std::size_t N>
        void ConstrainEqualTriplet(std::array<double, N> &diag,
                                   const std::size_t i,
                                   const std::size_t j,
                                   const std::size_t k)
        {
            const double v = (diag[i] + diag[j] + diag[k]) / 3.0;
            diag[i] = v;
            diag[j] = v;
            diag[k] = v;
        }

        FlightReconstruction::Measurement MakeMeasurement(const FlatPoint &fp,
                                                          const TrailPoint &p)
        {
            FlightReconstruction::Measurement measurement;
            auto &[y, x, z, U] = measurement.data;
            x.value = fp.x;
            y.value = fp.y;
            z.value = -p.pos.gps_altitude;
            U.value = p.v_tas;
            return measurement;
        }

        FlightReconstruction::State MakeInitialState(const FlatPoint &fp,
                                                     const TrailPoint &p)
        {
            return FlightReconstruction::get_initial_state_estimate(
                fp.y, fp.x, -p.pos.gps_altitude,
                p.v_tas,
                p.bank_angle.Radians(),
                p.pitch_angle.Radians(),
                p.yaw_angle.AsBearing().Radians());
        }

        Eigen::Vector3d QuaternionResidual(const FlightReconstruction::State &predicted,
                                           const FlightReconstruction::State &actual)
        {
            const auto &q_pred = std::get<1>(predicted.data).get_q();
            const auto &q_actual = std::get<1>(actual.data).get_q();

            Eigen::Quaterniond dq = q_pred.conjugate() * q_actual;
            if (dq.w() < 0.0)
                dq.coeffs() *= -1.0;

            const Eigen::Vector3d imag = dq.vec();
            const double imag_norm = imag.norm();
            if (imag_norm < 1e-12)
                return Eigen::Vector3d::Zero();

            const double angle = 2.0 * std::atan2(imag_norm, dq.w());
            return imag * (angle / imag_norm);
        }

        ProcessDiag GetBaselineProcess()
        {
            ProcessDiag out{};
            const auto defaults = FlightReconstruction::GetProcessCovarianceDefaults();
            for (std::size_t i = 0; i < out.size(); ++i)
                out[i] = defaults[i].second;
            return out;
        }

        MeasurementDiag GetBaselineMeasurement()
        {
            MeasurementDiag out{};
            const auto defaults = FlightReconstruction::GetMeasurementCovarianceDefaults();
            for (std::size_t i = 0; i < out.size(); ++i)
                out[i] = defaults[i].second;
            return out;
        }

        StateDiag GetBaselineState()
        {
            StateDiag out{};
            const auto defaults = FlightReconstruction::GetStateCovarianceDefaults();
            for (std::size_t i = 0; i < out.size(); ++i)
                out[i] = defaults[i].second;
            return out;
        }

        template <std::size_t N>
        boost::json::object NamedObject(const std::array<std::pair<std::string_view, double>, N> &defaults,
                                        const std::array<double, N> &diag)
        {
            boost::json::object obj;
            for (std::size_t i = 0; i < N; ++i)
                obj.emplace(defaults[i].first, diag[i]);
            return obj;
        }

    } // namespace

    CovarianceTuningResult TuneFlightReconstructionCovariances(
        const std::list<AircraftModel> &aircraft,
        const CovarianceTuningConfig &config)
    {
        const auto t_start = std::chrono::steady_clock::now();

        CovarianceTuningResult result;
        if (!config.enabled)
        {
            result.message = "disabled";
            return result;
        }

        std::cout << "[cov-tune] start"
                  << " total_aircraft=" << aircraft.size()
                  << " max_flights=" << config.max_flights
                  << " max_points_per_flight=" << config.max_points_per_flight
                  << " max_failures_per_flight=" << config.max_failures_per_flight
                  << " max_consecutive_failures=" << config.max_consecutive_failures
                  << " max_restarts_per_flight=" << config.max_restarts_per_flight
                  << " bootstrap_iterations=" << config.bootstrap_iterations
                  << " bootstrap_fraction=" << config.bootstrap_fraction
                  << " seed=" << config.random_seed
                  << "\n";

        ResidualPool<4> measurement_pool{};
        ResidualPool<9> process_pool{};
        ResidualPool<9> state_pool{};

        std::vector<const AircraftModel *> selected_aircraft;
        selected_aircraft.reserve(aircraft.size());
        for (const auto &a : aircraft)
        {
            if (a.GetTrail().size() >= 3)
                selected_aircraft.push_back(&a);
        }

        std::mt19937 rng(config.random_seed);
        if (config.max_flights > 0 && selected_aircraft.size() > config.max_flights)
        {
            std::shuffle(selected_aircraft.begin(), selected_aircraft.end(), rng);
            selected_aircraft.resize(config.max_flights);
        }

        std::cout << "[cov-tune] selected_flights=" << selected_aircraft.size() << "\n";

        std::size_t measurement_samples = 0;
        std::size_t process_samples = 0;
        std::size_t initial_state_samples = 0;
        std::size_t flights_processed = 0;

        for (const auto *ap : selected_aircraft)
        {
            const auto &a = *ap;
            const auto &trail = a.GetTrail();
            if (trail.size() < 3)
                continue;

            const std::size_t measurement_before = measurement_samples;
            const std::size_t process_before = process_samples;
            const std::size_t init_before = initial_state_samples;

            auto it = trail.begin();
            const GeoPoint origin = it->pos.location;
            FlatProjection projection(origin);

            const FlatPoint fp0 = projection.ProjectFloat(it->pos.location) * projection.GetApproximateScale();
            const auto initial_state = MakeInitialState(fp0, *it);

            FlightReconstruction::Filter filter;
            filter.initialise(initial_state, 1.0);

            bool have_previous_state = false;
            FlightReconstruction::State previous_state;
            std::size_t success_updates_this_flight = 0;
            std::size_t failures_this_flight = 0;
            std::size_t consecutive_failures = 0;
            std::size_t restarts_this_flight = 0;

            for (; it != trail.end(); ++it)
            {
                if (it->v_tas <= 0.0)
                    continue;

                if (config.max_points_per_flight > 0 &&
                    success_updates_this_flight >= config.max_points_per_flight)
                    break;

                if (config.max_failures_per_flight > 0 &&
                    failures_this_flight >= config.max_failures_per_flight)
                    break;

                if (config.max_consecutive_failures > 0 &&
                    consecutive_failures >= config.max_consecutive_failures)
                {
                    if (config.max_restarts_per_flight > 0 &&
                        restarts_this_flight < config.max_restarts_per_flight)
                    {
                        const FlatPoint fp_restart = projection.ProjectFloat(it->pos.location) * projection.GetApproximateScale();
                        const auto restart_state = MakeInitialState(fp_restart, *it);
                        filter.initialise(restart_state, 1.0);
                        have_previous_state = false;
                        consecutive_failures = 0;
                        ++restarts_this_flight;
                        std::cout << "[cov-tune] flight restart restarts=" << restarts_this_flight
                                  << " failures=" << failures_this_flight
                                  << " success_updates=" << success_updates_this_flight << "\n";
                    }
                    else
                    {
                        break;
                    }
                }

                const FlatPoint fp = projection.ProjectFloat(it->pos.location) * projection.GetApproximateScale();
                const auto measurement = MakeMeasurement(fp, *it);

                try
                {
                    filter.update(measurement, 1.0);
                }
                catch (const std::exception &)
                {
                    ++failures_this_flight;
                    ++consecutive_failures;
                    continue;
                }

                ++success_updates_this_flight;
                consecutive_failures = 0;

                const auto innovation = filter.get_innovation();
                for (int i = 0; i < innovation.rows(); ++i)
                    measurement_pool[static_cast<std::size_t>(i)].push_back(innovation(i));
                ++measurement_samples;

                const auto &state = filter.get_state();
                if (!have_previous_state)
                {
                    const auto init_deriv = FlightReconstruction::convert_state(initial_state);
                    const auto curr_deriv = FlightReconstruction::convert_state(state);
                    for (int i = 0; i < 6; ++i)
                        state_pool[static_cast<std::size_t>(i)].push_back(curr_deriv[i] - init_deriv[i]);

                    const Eigen::Vector3d dq = QuaternionResidual(initial_state, state);
                    state_pool[6].push_back(dq.x());
                    state_pool[7].push_back(dq.y());
                    state_pool[8].push_back(dq.z());
                    ++initial_state_samples;

                    previous_state = state;
                    have_previous_state = true;
                    continue;
                }

                auto predicted = previous_state;
                filter.system_model(predicted, 1.0);

                const auto pred_deriv = FlightReconstruction::convert_state(predicted);
                const auto curr_deriv = FlightReconstruction::convert_state(state);

                for (int i = 0; i < 6; ++i)
                    process_pool[static_cast<std::size_t>(i)].push_back(curr_deriv[i] - pred_deriv[i]);

                const Eigen::Vector3d dq = QuaternionResidual(predicted, state);
                process_pool[6].push_back(dq.x());
                process_pool[7].push_back(dq.y());
                process_pool[8].push_back(dq.z());
                ++process_samples;

                previous_state = state;
            }

            ++flights_processed;
            if ((flights_processed % 5) == 0 || flights_processed == selected_aircraft.size())
            {
                const auto now = std::chrono::steady_clock::now();
                const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - t_start).count();
                std::cout << "[cov-tune] flight-progress "
                          << flights_processed << "/" << selected_aircraft.size()
                          << " trail_points=" << trail.size()
                          << " success_updates=" << success_updates_this_flight
                          << " failures=" << failures_this_flight
                          << " restarts=" << restarts_this_flight
                          << " +meas=" << (measurement_samples - measurement_before)
                          << " +proc=" << (process_samples - process_before)
                          << " +init=" << (initial_state_samples - init_before)
                          << " elapsed_ms=" << elapsed_ms
                          << "\n";
            }
        }

        result.measurement_samples = measurement_samples;
        result.process_samples = process_samples;
        result.initial_state_samples = initial_state_samples;

        // initial_state_samples is bounded by the number of aircraft (one per
        // flight), so it uses a much lower floor than the per-timestep counters.
        const std::size_t min_samples = config.min_samples;
        if (measurement_samples < min_samples || process_samples < min_samples)
        {
            result.message = "insufficient samples for stable covariance tuning";
            std::cout << "[cov-tune] stop " << result.message
                      << " measurement=" << measurement_samples
                      << " process=" << process_samples
                      << " min_required=" << min_samples << "\n";
            return result;
        }
        if (initial_state_samples < 2)
        {
            result.message = "insufficient aircraft for initial state covariance estimation";
            std::cout << "[cov-tune] stop " << result.message
                      << " initial_state=" << initial_state_samples << "\n";
            return result;
        }

        const std::array<const char *, 4> measurement_labels{
            "R.x", "R.y", "R.z", "R.v_tas"};
        const std::array<const char *, 9> process_labels{
            "Q.x", "Q.y", "Q.z", "Q.u", "Q.w", "Q.q", "Q.attitude_x", "Q.attitude_y", "Q.attitude_z"};
        const std::array<const char *, 9> state_labels{
            "P.x", "P.y", "P.z", "P.u", "P.w", "P.q", "P.attitude_x", "P.attitude_y", "P.attitude_z"};

        std::cout << "[cov-tune] fitting diagonals\n";
        auto tuned_r = BuildDiagonal<4>(measurement_pool, GetBaselineMeasurement(), config, rng, measurement_labels);
        auto tuned_q = BuildDiagonal<9>(process_pool, GetBaselineProcess(), config, rng, process_labels);
        auto tuned_p = BuildDiagonal<9>(state_pool, GetBaselineState(), config, rng, state_labels);

        // Enforce symmetry constraints requested for tuned covariances.
        ConstrainEqualPair(tuned_r, 0, 1);       // measurement: x == y
        ConstrainEqualPair(tuned_q, 0, 1);       // process: x == y
        ConstrainEqualPair(tuned_p, 0, 1);       // state: x == y
        ConstrainEqualTriplet(tuned_q, 6, 7, 8); // process: attitude_x == attitude_y == attitude_z
        ConstrainEqualTriplet(tuned_p, 6, 7, 8); // state: attitude_x == attitude_y == attitude_z

        const auto process_defaults = FlightReconstruction::GetProcessCovarianceDefaults();
        const auto measurement_defaults = FlightReconstruction::GetMeasurementCovarianceDefaults();
        const auto state_defaults = FlightReconstruction::GetStateCovarianceDefaults();

        boost::json::object root;
        root.emplace("method", "innovation_and_residual_variance_bootstrap");
        root.emplace("fit", boost::json::object{
                                {"selected_flights", selected_aircraft.size()},
                                {"max_flights", config.max_flights},
                                {"max_points_per_flight", config.max_points_per_flight},
                                {"bootstrap_iterations", config.bootstrap_iterations},
                                {"bootstrap_fraction", config.bootstrap_fraction},
                                {"random_seed", config.random_seed},
                            });
        root.emplace("samples", boost::json::object{
                                    {"measurement", measurement_samples},
                                    {"process", process_samples},
                                    {"initial_state", initial_state_samples},
                                });
        root.emplace("process_covariance", NamedObject(process_defaults, tuned_q));
        root.emplace("measurement_covariance", NamedObject(measurement_defaults, tuned_r));
        root.emplace("state_covariance", NamedObject(state_defaults, tuned_p));

        result.success = true;
        result.covariances = std::move(root);
        result.message = "ok";

        const auto t_end = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        std::cout << "[cov-tune] done elapsed_ms=" << elapsed_ms
                  << " measurement_samples=" << measurement_samples
                  << " process_samples=" << process_samples
                  << " initial_state_samples=" << initial_state_samples << "\n";

        return result;
    }

} // namespace MultiAircraft
