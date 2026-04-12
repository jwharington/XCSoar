#include "FlightReconstructionInternal.hpp"

namespace FlightReconstruction
{
    DerivState Filter::system_ode(const DerivState &state) const
    {
        const auto &u = state[VEL_U];
        const auto &w = state[VEL_W];
        const auto &q = state[AVEL_Q];
        const auto &q0 = state[QUATERNION + 0];
        const auto &q1 = state[QUATERNION + 1];
        const auto &q2 = state[QUATERNION + 2];
        const auto &q3 = state[QUATERNION + 3];

        const AeroLoad aero(state, parms);

        const Eigen::Matrix3d R =
            Eigen::Quaterniond(q0, q1, q2, q3).toRotationMatrix();
        const Eigen::Vector3d pos_dot = R * Eigen::Vector3d(u, 0, w);

        const auto p = 0.0;
        const auto qdot = 0;
        const auto r = aero.env.g * R(2, 1) / u;

        const auto udot = -q * w + R(2, 0) * aero.env.g + aero.ax;
        const auto wdot = q * u + R(2, 2) * aero.env.g + aero.az;

        const auto qmag = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
        const auto lam = 1 - qmag;
        const auto q0dot = -0.5 * (q1 * p + q2 * q + q3 * r) + lam * q0;
        const auto q1dot = 0.5 * (q0 * p + q2 * r - q3 * q) + lam * q1;
        const auto q2dot = 0.5 * (q0 * q - q1 * r + q3 * p) + lam * q2;
        const auto q3dot = 0.5 * (q0 * r + q1 * q - q2 * p) + lam * q3;

        return DerivState({pos_dot(0), pos_dot(1), pos_dot(2),
                           udot, wdot, qdot,
                           q0dot, q1dot, q2dot, q3dot});
    }

    void Filter::system_model(State &state, double dt) const
    {
        detail::SystemModel(state, dt, [this](const DerivState &deriv_state)
                            { return system_ode(deriv_state); }, [](const State &source_state)
                            { return detail::ConvertState<6, QUATERNION>(source_state); }, [](State &target_state, const DerivState &deriv_state)
                            { detail::SetState<6, QUATERNION>(target_state, deriv_state); });
    }

    void Filter::limit_state(State &state) const
    {
        detail::LimitState(state);
    }

    Measurement Filter::measurement_model(const State &state) const
    {
        return detail::MeasurementModel(state);
    }

    void Filter::update(const Measurement &meas, const double DT)
    {
        detail::Update<Filter, UKF, State>(*this, ukf, meas, DT);
    }

    void Filter::initialise(const State &initial_state_estimate,
                            const double DT)
    {
        (void)DT;
        detail::Initialise(ukf, initial_state_estimate, RTS_WINDOW_SIZE,
                           detail::ProcessCovarianceDefaultsStorage(),
                           detail::MeasurementCovarianceDefaultsStorage(),
                           detail::StateCovarianceDefaultsStorage());
    }

    const AeroLoad Filter::get_aero() const
    {
        return get_aero(get_state());
    }

    const AeroLoad Filter::get_aero(const State &state) const
    {
        return detail::GetAero<6, QUATERNION>(state, parms);
    }

    const Euler Filter::get_euler() const
    {
        return detail::GetEuler(get_state());
    }
}