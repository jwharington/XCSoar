#include "GliderAeroLoads.hpp"
#include "FlightReconstruction.hpp"

namespace FlightReconstruction
{
    // Standard Constants
    static constexpr double P0 = 101325.0; // Sea level pressure (Pa)
    static const double T0 = 288.15;       // Sea level temperature (K)
    static const double g0 = 9.80665;      // Gravity (m/s^2)
    static const double L = 0.0065;        // Temperature lapse rate (K/m)
    static const double R = 287.058;       // Gas constant for dry air (J/(kg*K))
    static const double rho0 = 1.225;      // Sea level density (kg/m^3)

    static double getISADensity(double h_m)
    {
        const double T = T0 - L * h_m;
        return rho0 * std::pow(T / T0, (g0 / (L * R)) - 1.0);
    };

    AeroLoad::AeroLoad(const DerivState &state, const GliderAero &parms)
    {
        auto &u = state[VEL_U];
        auto &w = state[VEL_W];
        auto &z = state[POS_Z];
        V = hypot(u, w);
        alpha = atan2(w, u);
        rho = getISADensity(-z);
        g = g0;

        double CL, CD;
        parms.calc_CLCD(alpha, CL, CD);

        const double QS_mV = 0.5 * V * parms.S / parms.m;
        const auto sa = w * QS_mV; // sin(alpha) QS/m
        const auto ca = u * QS_mV; // cos(alpha) QS/m
        ax = CL * sa - CD * ca;
        az = -CL * ca - CD * sa;
        load_factor = -sign(az) * hypot(ax, az) / g;
    };
};
