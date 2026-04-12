#include "FlightReconstructionInternal.hpp"

namespace FlightReconstruction
{
    DerivState FilterWithUpdraftGust::system_ode(const DerivState &state) const
    {
        const auto &u = state[VEL_U];
        const auto &w = state[VEL_W];
        const auto &q = state[AVEL_Q];
        const auto &w_g = state[detail::WG_INDEX];
        const auto &q0 = state[detail::QUATERNION_WITH_UPDRAFT_GUST + 0];
        const auto &q1 = state[detail::QUATERNION_WITH_UPDRAFT_GUST + 1];
        const auto &q2 = state[detail::QUATERNION_WITH_UPDRAFT_GUST + 2];
        const auto &q3 = state[detail::QUATERNION_WITH_UPDRAFT_GUST + 3];

        const AeroLoad aero(state, parms);

        const Eigen::Matrix3d R =
            Eigen::Quaterniond(q0, q1, q2, q3).toRotationMatrix();
        const Eigen::Vector3d pos_dot = R * Eigen::Vector3d(u, 0, w);

        const auto p = 0.0;
        const auto qdot = 0;
        const auto r = aero.env.g * R(2, 1) / u;

        const auto udot = -q * w + R(2, 0) * aero.env.g + aero.ax;
        const auto wdot = q * u + R(2, 2) * aero.env.g + aero.az;
        const auto w_g_dot = 0;

        const auto qmag = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
        const auto lam = 1 - qmag;
        const auto q0dot = -0.5 * (q1 * p + q2 * q + q3 * r) + lam * q0;
        const auto q1dot = 0.5 * (q0 * p + q2 * r - q3 * q) + lam * q1;
        const auto q2dot = 0.5 * (q0 * q - q1 * r + q3 * p) + lam * q2;
        const auto q3dot = 0.5 * (q0 * r + q1 * q - q2 * p) + lam * q3;

        return DerivState({pos_dot(0), pos_dot(1), pos_dot(2) + w_g,
                           udot, wdot, qdot, w_g_dot,
                           q0dot, q1dot, q2dot, q3dot});
    }

    void FilterWithUpdraftGust::system_model(StateWithUpdraftGust &state, double dt) const
    {
        detail::SystemModel(state, dt, [this](const DerivState &deriv_state)
                            { return system_ode(deriv_state); }, [](const StateWithUpdraftGust &source_state)
                            { return detail::ConvertState<7, detail::QUATERNION_WITH_UPDRAFT_GUST>(source_state); }, [](StateWithUpdraftGust &target_state, const DerivState &deriv_state)
                            { detail::SetState<7, detail::QUATERNION_WITH_UPDRAFT_GUST>(target_state, deriv_state); });
    }

    void FilterWithUpdraftGust::limit_state(StateWithUpdraftGust &state) const
    {
        detail::LimitState(state);
    }

    Measurement FilterWithUpdraftGust::measurement_model(const StateWithUpdraftGust &state) const
    {
        return detail::MeasurementModel(state);
    }

    void FilterWithUpdraftGust::update(const Measurement &meas, const double DT)
    {
        detail::Update<FilterWithUpdraftGust, UKFWithUpdraftGust, StateWithUpdraftGust>(*this, ukf, meas, DT);
    }

    void FilterWithUpdraftGust::initialise(const StateWithUpdraftGust &initial_state_estimate,
                                           const double DT)
    {
        (void)DT;
        detail::Initialise(ukf, initial_state_estimate, RTS_WINDOW_SIZE,
                           detail::ProcessCovarianceDefaultsWithUpdraftGustStorage(),
                           detail::MeasurementCovarianceDefaultsStorage(),
                           detail::StateCovarianceDefaultsWithUpdraftGustStorage());
    }

    const AeroLoad FilterWithUpdraftGust::get_aero() const
    {
        return get_aero(get_state());
    }

    const AeroLoad FilterWithUpdraftGust::get_aero(const StateWithUpdraftGust &state) const
    {
        return detail::GetAero<7, detail::QUATERNION_WITH_UPDRAFT_GUST>(state, parms);
    }

    const Euler FilterWithUpdraftGust::get_euler() const
    {
        return detail::GetEuler(get_state());
    }

    StateWithUpdraftGust get_initial_state_estimate_with_updraft_gust(
        const double x, const double y, const double z,
        const double U, const double bank,
        const double pitch, const double hdg,
        const double w_g)
    {
        Eigen::Quaterniond init_quaternion =
            Eigen::AngleAxisd(hdg, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(bank, Eigen::Vector3d::UnitX());
        return StateWithUpdraftGust({x, y, z, U, 0.0, 0.0, w_g},
                                    unscented::UnitQuaternion(init_quaternion));
    }
}