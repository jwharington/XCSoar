#pragma once
#include "unscented/primitives.h"
#include "unscented/ukf.h"

namespace FlightReconstruction
{
    static constexpr double DEGTORAD = M_PI / 180.0;

    template <typename T>
    int sign(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    // x,y, z, u, w, q, q0, q1, q2, q3
    enum StateElements
    {
        POS_X = 0,
        POS_Y,
        POS_Z,
        VEL_U,
        VEL_W,
        AVEL_Q,
        QUATERNION,
    };

    using State =
        unscented::Compound<unscented::Vector<6>, unscented::UnitQuaternion>;

    using DerivState = std::vector<double>;

    using Pos = unscented::Scalar;
    using Vel = unscented::Scalar;
    using Measurement = unscented::Compound<Pos, Pos, Pos, Vel>;

    using UKF = unscented::UKF<State, Measurement>;

    struct GliderAero
    {
        double S = 10.0;
        double dcldalpha = 6.0;
        double k = 0.00769772063821893;
        double g = 9.81;
        double CL0 = 0.5248831739026187;
        double CD0 = 0.0107362467389172;
        double m = 500.0;
        double rho = 1.225; // air density kg/m^3
    };

    struct AeroLoad
    {
        double V;
        double alpha;
        double ax;
        double az;
        double load_factor;

        AeroLoad(const DerivState &state, const GliderAero &parms);
    };

    // {'LD_best': 55,
    //  'V_LDbest': 30.8641975308642,
    //  'V_cruise': 20.5761316872428,
    //  'S': 10.0,
    //  'm': 500.0,
    //  'rho': 1.225,
    //  'g': 9.881,
    //  'dcldalpha': 6.0,
    //  'wind_n': 2.5308641975308643,
    //  'wind_e': -11.882716049382717,
    //  'k': 0.00769772063821893,
    //  'CD0': 0.0107362467389172,
    //  'CL0': 0.5248831739026187}

    class Filter
    {
    public:
        void update(const Measurement &meas, const double DT);
        DerivState system_ode(const DerivState &state) const;
        void system_model(State &state, double dt) const;
        Measurement measurement_model(const State &state) const;
        void initialise(const State &initial_state_estimate, const double DT);
        void limit_state(State &state) const;
        const State &get_state() const { return ukf.get_state(); };
        const AeroLoad get_aero() const;
        const Eigen::Vector3d get_euler() const;

    private:
        UKF ukf;
        GliderAero parms;
    };

    State get_initial_state_estimate(const double x, const double y, const double z,
                                     const double U, const double hdg,
                                     const double pitch, const double bank);

};
