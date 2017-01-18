#pragma once

namespace MultiAircraft {

struct Averager {
  bool empty() const {
    return num==0;
  }
  void add(const Averager &a) {
    acc+= a.acc;
    num+= a.num;
  }
  void add(const double x) {
    acc+= x;
    num++;
  }
  bool calculate() {
    if (num) {
      avg = acc/num;
      return true;
    } else {
      return false;
    }
  }
  double get_avg() const {
    return avg;
  }
 private:
  int num = 0;
  double acc = 0;
  double avg = 0;
};

}
