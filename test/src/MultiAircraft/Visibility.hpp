#pragma once

#include <math.h>
#include "Math/Angle.hpp"

namespace MultiAircraft {

struct TrigAngle {
  TrigAngle(const Angle _a):a(_a) {
    const auto i = a.SinCos();
    s = i.first;
    c = i.second;
  }
  Angle a;
  double s;
  double c;
};

struct Aspect {
  Aspect() = default;
  Aspect(const double x_body[3], const double x_inertial[3]) {
    range = sqrt(x_body[0]*x_body[0]+x_body[1]*x_body[1]+x_body[2]*x_body[2]);

    elevation_angle = Angle::asin(-x_body[2]/range);
    azimuth_angle = Angle::FromXY(x_body[0], x_body[1]);
    inclination_angle = Angle::asin(-x_inertial[2]/range);
  }
  double range = -1;
  Angle elevation_angle = Angle::Native(0);
  Angle azimuth_angle = Angle::Native(0);
  Angle inclination_angle = Angle::Native(0);
};

struct Visibility {
  Visibility() = default;
  Visibility(const Aspect& aspect);
  Angle angular_size = Angle::Native(0);
  double occlusion = 0;
  double focus_factor = 0;
};

struct EulerAngles {
  EulerAngles(): EulerAngles(Angle::Native(0), Angle::Native(0), Angle::Native(0)) {}

  EulerAngles(const Angle _phi, const Angle _theta, const Angle _psi): phi(_phi), theta(_theta), psi(_psi)
  {
    R[0][0] = psi.c * theta.c;
    R[0][1] = theta.c * psi.s;
    R[0][2] = -theta.s;
    R[1][0] = psi.c * phi.s * theta.s - phi.c * psi.s;
    R[1][1] = phi.c * psi.c + phi.s * psi.s * theta.s;
    R[1][2] = theta.c * phi.s;
    R[2][0] = phi.s * psi.s + phi.c * psi.c * theta.s;
    R[2][1] = phi.c * psi.s * theta.s - psi.c * phi.s;
    R[2][2] = phi.c * theta.c;
  }

  TrigAngle phi;
  TrigAngle theta;
  TrigAngle psi;
  double R[3][3];

  //    x north, y east, z down
  Aspect get_aspect(const double x_inertial[3]) const
  {
    const double x_body[3] = {
      R[0][0] * x_inertial[0] + R[0][1] * x_inertial[1] + R[0][2] * x_inertial[2],
      R[1][0] * x_inertial[0] + R[1][1] * x_inertial[1] + R[1][2] * x_inertial[2],
      R[2][0] * x_inertial[0] + R[2][1] * x_inertial[1] + R[2][2] * x_inertial[2]
    };
    return Aspect(x_body, x_inertial);
  }
};

}

void test_visibility();
