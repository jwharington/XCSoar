#pragma once
#include <vector>

namespace FlightReconstruction
{
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
    using DerivState = std::vector<double>;

    struct GliderAero
    {
        double S = 10.0;
        double dcldalpha = 6.0;
        double k = 0.00769772063821893;
        double CL0 = 0.5248831739026187;
        double CD0 = 0.0107362467389172;
        double m = 500.0;
        double CLmax = 1.5;

        void calc_CLCD(const double alpha, double &CL, double &CD) const
        {
            CL = CL0 + dcldalpha * alpha;
            CD = CD0 + k * CL * CL;
            CL = std::min(std::max(CL, -CLmax), CLmax);
        };
    };

    struct AeroLoad
    {
        double V;
        double alpha;
        double ax;
        double az;
        double load_factor;
        double g;
        double rho;

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
};