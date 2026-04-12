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

    using Pos = unscented::Scalar;
    using Vel = unscented::Scalar;
    using Measurement = unscented::Compound<Pos, Pos, Pos, Vel>;

    using UKF = unscented::UKF<State, Measurement>;
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

    private:
        UKF ukf;
        GliderAero parms;
    };

    State get_initial_state_estimate(const double x, const double y, const double z,
                                     const double U, const double hdg,
                                     const double pitch, const double bank);

};
