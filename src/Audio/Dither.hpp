#pragma once

struct DitherRNG {
  DitherRNG(const unsigned short a, const unsigned short X0): a(a),X(X0) {
  }
  const unsigned short a;
  unsigned short X;

  unsigned short update() {
    X = a*X+1;
    return X;
  }
};

struct Dither {
  Dither(): d1(3453,1531), d2(2945,18531) {};
  unsigned short update() {
    return ((d1.update()>>1) + (d2.update()>>1))>>1;
  }

  DitherRNG d1;
  DitherRNG d2;
};
