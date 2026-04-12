#pragma once
#include "unscented/primitives.h"
#include "unscented/ukf.h"
#include "GliderAeroLoads.hpp"

namespace FlightReconstruction
{
    static constexpr double DEGTORAD = M_PI / 180.0;

    template <typename T>
    int sign(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    using State =
        unscented::Compound<unscented::Vector<6>, unscented::UnitQuaternion>;
    using StateWithUpdraftGust =
        unscented::Compound<unscented::Vector<7>, unscented::UnitQuaternion>;

    using Pos = unscented::Scalar;
    using Vel = unscented::Scalar;
    using Measurement = unscented::Compound<Pos, Pos, Pos, Vel>;

    using UKF = unscented::UKF<State, Measurement>;
    using UKFWithUpdraftGust = unscented::UKF<StateWithUpdraftGust, Measurement>;
    using Euler = Eigen::Vector3d;

    class Filter
    {
    public:
        static unsigned RTS_WINDOW_SIZE;

        void update(const Measurement &meas, const double DT);
        DerivState system_ode(const DerivState &state) const;
        void system_model(State &state, double dt) const;
        Measurement measurement_model(const State &state) const;
        void initialise(const State &initial_state_estimate, const double DT);
        void limit_state(State &state) const;
        const State &get_state() const { return ukf.get_state(); };
        const AeroLoad get_aero() const;
        const AeroLoad get_aero(const State &state) const;
        const Euler get_euler() const;
        const std::vector<State> &get_smoothed_states() const { return ukf.get_smoothed_states(); };
        const UKF::M_by_1 &get_innovation() const { return ukf.get_innovation(); }

    private:
        UKF ukf;
        GliderAero parms;
    };

    class FilterWithUpdraftGust
    {
    public:
        static unsigned RTS_WINDOW_SIZE;

        void update(const Measurement &meas, const double DT);
        DerivState system_ode(const DerivState &state) const;
        void system_model(StateWithUpdraftGust &state, double dt) const;
        Measurement measurement_model(const StateWithUpdraftGust &state) const;
        void initialise(const StateWithUpdraftGust &initial_state_estimate, const double DT);
        void limit_state(StateWithUpdraftGust &state) const;
        const StateWithUpdraftGust &get_state() const { return ukf.get_state(); };
        const AeroLoad get_aero() const;
        const AeroLoad get_aero(const StateWithUpdraftGust &state) const;
        const Euler get_euler() const;
        const std::vector<StateWithUpdraftGust> &get_smoothed_states() const { return ukf.get_smoothed_states(); };
        const UKFWithUpdraftGust::M_by_1 &get_innovation() const { return ukf.get_innovation(); }

    private:
        UKFWithUpdraftGust ukf;
        GliderAero parms;
    };

    State get_initial_state_estimate(const double x, const double y, const double z,
                                     const double U, const double hdg,
                                     const double pitch, const double bank);
    StateWithUpdraftGust get_initial_state_estimate_with_updraft_gust(
        double x, double y, double z,
        double U, double bank,
        double pitch, double hdg,
        double w_g = 0.0);

};
