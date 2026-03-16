#include "FlightReconstruction.hpp"
#include "ReconstructionUtility.hpp"

#include "DOP853.h"
using namespace tableau::integration;

namespace FlightReconstruction
{

  DerivState Filter::system_ode(const DerivState &state) const
  {
    auto &u = state[VEL_U];
    auto &w = state[VEL_W];
    auto &q = state[AVEL_Q];
    auto &q0 = state[QUATERNION + 0];
    auto &q1 = state[QUATERNION + 1];
    auto &q2 = state[QUATERNION + 2];
    auto &q3 = state[QUATERNION + 3];

    const auto V_sq = u * u + w * w;
    const auto V = sqrt(V_sq);
    const auto alpha = atan(w / u);

    auto CL = parms.dcldalpha * alpha + parms.CL0;
    // CL = sympy.Min(1.5, sympy.Max(0.0, cl_raw))
    auto CD = parms.CD0 + parms.k * CL * CL;
    // CLL = sympy.Min(CL, 1.0)
    auto CLL = CL;

    const auto Q = 0.5 * parms.rho * V_sq;
    const auto QS_m = Q * parms.S / parms.m;
    const auto sa = w / V; // sin(alpha)
    const auto ca = u / V; // cos(alpha)
    const auto Cx = CLL * sa - CD * ca;
    const auto Cz = -CLL * ca - CD * sa;

    // orientation
    const Eigen::Matrix3d R =
        Eigen::Quaterniond(q0, q1, q2, q3).toRotationMatrix();
    const Eigen::Vector3d U(u, 0, w);
    const Eigen::Vector3d pos_dot = R * U;

    const auto p = 0.0;
    const auto qdot = 0;
    // vdot = -r * u + g * b3 = 0
    const auto r = parms.g * R(2, 1) / u;

    auto udot = -q * w + R(2, 0) * parms.g + QS_m * Cx;
    auto wdot = q * u + R(2, 2) * parms.g + QS_m * Cz;

    auto qmag = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    auto lam = 1 - qmag;
    auto q0dot = -0.5 * (q1 * p + q2 * q + q3 * r) + lam * q0;
    auto q1dot = 0.5 * (q0 * p + q2 * r - q3 * q) + lam * q1;
    auto q2dot = 0.5 * (q0 * q - q1 * r + q3 * p) + lam * q2;
    auto q3dot = 0.5 * (q0 * r + q1 * q - q2 * p) + lam * q3;

    return DerivState({pos_dot(0), pos_dot(1), pos_dot(2), udot, wdot, qdot,
                       q0dot, q1dot, q2dot, q3dot});
  }

  void Filter::system_model(State &state, double dt) const
  {
    double t0 = 0.0;
    DerivState y0 = convert_state(state);
    DerivState dy0 = system_ode(y0);

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
    auto &u = states[VEL_U];
    auto &w = states[VEL_W];
    auto V_sq = u * u + w * w;
    auto V = sqrt(V_sq);
    return {states[POS_X], states[POS_Y], states[POS_Z], V};
  }

  void Filter::update(const Measurement &meas, const double DT)
  {
    ukf.predict([this](State &state, double dt)
                { this->system_model(state, dt); }, DT);
    ukf.correct([this](const State &state)
                { return this->measurement_model(state); }, meas);
    ukf.smooth();
    State state = ukf.get_state();
    limit_state(state);
    ukf.set_state(state);
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

};
