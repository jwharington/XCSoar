#include "FlightReconstruction.hpp"
#include "ReconstructionUtility.hpp"

#include "DOP853.h"
using namespace tableau::integration;

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

    // orientation
    const Eigen::Matrix3d R =
        Eigen::Quaterniond(q0, q1, q2, q3).toRotationMatrix();
    const Eigen::Vector3d pos_dot = R * Eigen::Vector3d(u, 0, w);

    // rotation conditions
    const auto p = 0.0;
    const auto qdot = 0;
    // vdot = -r * u + g * b3 = 0  (balanced turn)
    const auto r = aero.env.g * R(2, 1) / u;

    // accelerations
    const auto udot = -q * w + R(2, 0) * aero.env.g + aero.ax;
    const auto wdot = q * u + R(2, 2) * aero.env.g + aero.az;

    // orientation rates (quaternions)
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
    const double t0 = 0.0;
    const DerivState y0 = convert_state(state);
    const DerivState dy0 = system_ode(y0);

    DOP853Config<DerivState> cfg;
    cfg.derivative = [this](const DerivState &y, double)
    { return this->system_ode(y); };
    DOP853Integrator<DerivState> integrator(cfg);
    auto result = integrator.integrate(t0, y0, dt,
                                       DOP853Tolerance::scalar(1.0e-12, 1.0e-12));
    set_state(state, result.y);
    limit_state(state);
  }

  void Filter::limit_state(State &state) const
  {
    auto &[states, attitude] = state.data;
    auto &u = states[VEL_U];
    if (u < 1.0)
    {
      u = 1.0;
    }
  }

  Measurement Filter::measurement_model(const State &state) const
  {
    auto &[states, attitude] = state.data;
    return {states[POS_X], states[POS_Y], states[POS_Z],
            hypot(states[VEL_U], states[VEL_W])};
  }

  void Filter::update(const Measurement &meas, const double DT)
  {
    ukf.predict([this](State &state, double dt)
                { this->system_model(state, dt); }, DT);
    ukf.correct([this](const State &state)
                { return this->measurement_model(state); }, meas);
    ukf.smooth();
    {
      State state = ukf.get_state();
      limit_state(state);
      ukf.set_state(state);
    }
  }

  void Filter::initialise(const State &initial_state_estimate,
                          const double DT)
  {
    (void)DT; // unused
    ukf.set_weight_coefficients(0.1, 2.0, -1.0);

    UKF::N_by_N Q;
    Q.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0, 0.01, 0.01, 0.01, 0.01;
    ukf.set_process_covariance(Q);

    UKF::M_by_M R;
    R.diagonal() << 4.0, 4.0, 4.0, 1.0;
    ukf.set_measurement_covariance(R);

    ukf.set_state(initial_state_estimate);
    UKF::N_by_N P;
    P.diagonal() << 10.0, 10.0, 10.0, 1.0, 1.0, 0.1, 1.0, 1.0, 1.0;
    ukf.set_state_covariance(P);
  }

  const Euler Filter::get_euler() const
  {
    return FlightReconstruction::get_euler(get_state());
  };

};
