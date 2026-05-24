#include "GliderAeroLoads.hpp"
#include "FlightReconstruction.hpp"

namespace FlightReconstruction
{
    // Standard Constants
    static constexpr double T0 = 288.15;   // Sea level temperature (K)
    static constexpr double g0 = 9.80665;  // Gravity (m/s^2)
    static constexpr double L = 0.0065;    // Temperature lapse rate (K/m)
    static constexpr double R = 287.058;   // Gas constant for dry air (J/(kg*K))
    static constexpr double rho0 = 1.225;  // Sea level density (kg/m^3)

    static constexpr double getISADensity(double h_m)
    {
        const double T = T0 - L * h_m;
        return std::pow(T / T0, (g0 / (L * R)) - 1.0);
    };

    Environment::Environment(const DerivState &state)
    {
        // local gravity
        g = g0;

        // density
        auto &z = state[POS_Z];
        rho_rat = getISADensity(-z);
        rho = rho0 * rho_rat;
    }

    AeroLoad::AeroLoad(const DerivState &state, const GliderAero &parms) : env(state)
    {
        // airspeeds
        auto &u = state[VEL_U];
        auto &w = state[VEL_W];
        V_tas = hypot(u, w);
        V_ias = V_tas * sqrt(env.rho_rat);

        // angle of attack and aero model
        alpha = atan2(w, u);
        double CL, CD;
        parms.calc_CLCD(alpha, CL, CD);

        // resolve aero forces in body axes
        const double QS_mV = 0.5 * env.rho * V_tas * parms.S / parms.m;
        const auto sa = w * QS_mV; // sin(alpha) QS/m
        const auto ca = u * QS_mV; // cos(alpha) QS/m
        ax = CL * sa - CD * ca;
        az = -CL * ca - CD * sa;

        // utility
        load_factor = -sign(az) * hypot(ax, az) / env.g;
    };
};
