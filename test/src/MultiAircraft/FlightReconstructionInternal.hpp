#pragma once

#include "ReconstructionUtility.hpp"
#include "DOP853.h"

#include <array>
#include <cmath>
#include <iostream>
#include <string_view>

namespace FlightReconstruction::detail
{
    using NamedDefaults9 = std::array<std::pair<std::string_view, double>, 9>;
    using NamedDefaults10 = std::array<std::pair<std::string_view, double>, 10>;
    using MeasurementDefaults = std::array<std::pair<std::string_view, double>, 4>;

    inline constexpr int WG_INDEX = 6;
    inline constexpr int QUATERNION_WITH_UPDRAFT_GUST = 7;

    NamedDefaults9 &ProcessCovarianceDefaultsStorage();
    NamedDefaults10 &ProcessCovarianceDefaultsWithUpdraftGustStorage();
    MeasurementDefaults &MeasurementCovarianceDefaultsStorage();
    NamedDefaults9 &StateCovarianceDefaultsStorage();
    NamedDefaults10 &StateCovarianceDefaultsWithUpdraftGustStorage();

    template <int LinearStateSize, int QuaternionIndex, typename StateType>
    DerivState ConvertState(const StateType &state)
    {
        auto &[y, quaternion] = state.data;
        DerivState vstate;
        vstate.reserve(LinearStateSize + 4);
        for (int index = 0; index < LinearStateSize; ++index)
        {
            vstate.push_back(y[index]);
        }

        auto &q = quaternion.get_q();
        vstate.push_back(q.w());
        vstate.push_back(q.x());
        vstate.push_back(q.y());
        vstate.push_back(q.z());
        (void)QuaternionIndex;
        return vstate;
    }

    template <int LinearStateSize, int QuaternionIndex, typename StateType>
    void SetState(StateType &state, const DerivState &vstate)
    {
        auto &[y, quaternion] = state.data;
        for (int index = 0; index < LinearStateSize; ++index)
        {
            y[index] = vstate[index];
        }

        quaternion = unscented::UnitQuaternion(
            Eigen::Quaterniond(vstate[QuaternionIndex + 0],
                               vstate[QuaternionIndex + 1],
                               vstate[QuaternionIndex + 2],
                               vstate[QuaternionIndex + 3]));
    }

    template <typename StateType>
    Euler GetEuler(const StateType &state)
    {
        auto &[y, quaternion] = state.data;
        (void)y;
        const Eigen::Matrix3d R = quaternion.get_q().toRotationMatrix();
        double theta = asin(-R(2, 0));
        double psi = acos(R(0, 0) / cos(theta)) * sign(R(1, 0));
        double phi = acos(R(2, 2) / cos(theta)) * sign(R(2, 1));
        if (psi < 0)
        {
            psi += 2 * M_PI;
        }

        return Euler(phi, theta, psi) / DEGTORAD;
    }

    template <typename StateType>
    void LimitState(StateType &state)
    {
        auto &[states, attitude] = state.data;
        (void)attitude;
        auto &u = states[VEL_U];
        if (u < 1.0)
        {
            u = 1.0;
        }
    }

    template <typename StateType>
    Measurement MeasurementModel(const StateType &state)
    {
        auto &[states, attitude] = state.data;
        (void)attitude;
        return {states[POS_X], states[POS_Y], states[POS_Z],
                hypot(states[VEL_U], states[VEL_W])};
    }

    template <typename StateType, typename SystemOdeFn, typename ConvertStateFn,
              typename SetStateFn>
    void SystemModel(StateType &state, double dt,
                     SystemOdeFn system_ode,
                     ConvertStateFn convert_state,
                     SetStateFn set_state)
    {
        using namespace tableau::integration;

        const double t0 = 0.0;
        const DerivState y0 = convert_state(state);

        DOP853Config<DerivState> cfg;
        cfg.derivative = [system_ode](const DerivState &y, double)
        { return system_ode(y); };
        cfg.validator = [](const DerivState &s)
        {
            for (const auto &v : s)
                if (!std::isfinite(v))
                    return false;
            return true;
        };
        DOP853Integrator<DerivState> integrator(cfg);
        auto result = integrator.integrate(t0, y0, dt,
                                           DOP853Tolerance::scalar(1.0e-12, 1.0e-12));
        if (result.status == DOP853Status::Success)
        {
            set_state(state, result.y);
        }
        // else: leave state unchanged (last known good)
        LimitState(state);
    }

    template <typename StateType>
    bool IsStateFinite(const StateType &state)
    {
        // Subtract state from itself to get its DOF vector representation
        // (all zeros if finite, NaN if any component is NaN)
        auto diff = state - state;
        return diff.allFinite();
    }

    template <typename FilterType, typename UKFType, typename StateType>
    void Update(FilterType &filter, UKFType &ukf,
                const Measurement &meas, double dt)
    {
        const StateType state_before = ukf.get_state();
        const typename UKFType::N_by_N P_before = ukf.get_state_covariance();

        ukf.predict([&filter](StateType &state, double delta_t)
                    { filter.system_model(state, delta_t); }, dt);
        ukf.correct([&filter](const StateType &state)
                    { return filter.measurement_model(state); }, meas);
        ukf.smooth();

        StateType state = ukf.get_state();

        // Check for NaN in updated state; if found, revert
        if (!IsStateFinite(state))
        {
            ukf.set_state(state_before);
            ukf.set_state_covariance(P_before);
            state = state_before;
            std::cerr << "[filter] NaN detected after correct(); reverted state"
                      << std::endl;
        }

        LimitState(state);
        ukf.set_state(state);
    }

    template <typename UKFType, typename StateType,
              std::size_t ProcessCount, std::size_t StateCount>
    void Initialise(UKFType &ukf,
                    const StateType &initial_state_estimate,
                    unsigned rts_window_size,
                    const std::array<std::pair<std::string_view, double>, ProcessCount> &process_defaults,
                    const MeasurementDefaults &measurement_defaults,
                    const std::array<std::pair<std::string_view, double>, StateCount> &state_defaults)
    {
        ukf.set_max_smoothing_points(rts_window_size);
        ukf.set_weight_coefficients(1.0, 2.0, 0.0);

        typename UKFType::N_by_N Q;
        SetDiagonalFromNamedDefaults(Q, process_defaults);
        ukf.set_process_covariance(Q);

        typename UKFType::M_by_M R;
        SetDiagonalFromNamedDefaults(R, measurement_defaults);
        ukf.set_measurement_covariance(R);

        ukf.set_state(initial_state_estimate);
        typename UKFType::N_by_N P;
        SetDiagonalFromNamedDefaults(P, state_defaults);
        ukf.set_state_covariance(P);
    }

    template <int LinearStateSize, int QuaternionIndex, typename StateType>
    AeroLoad GetAero(const StateType &state, const GliderAero &parms)
    {
        return AeroLoad(ConvertState<LinearStateSize, QuaternionIndex>(state), parms);
    }
}