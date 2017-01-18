#pragma once
#include <list>
#include <set>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cassert>

namespace Flock {

struct Point {
  Point(double _x, double _y): x(_x), y(_y) {};
  double x;
  double y;
  double dist_sq(const Point& other) const {
    return (x-other.x)*(x-other.x) + (y-other.y)*(y-other.y);
  }

};


struct IndexPoint: public Point
{
  IndexPoint(Point _p, int _i): Point(_p), index(_i) {};

  bool operator<( const IndexPoint& other ) const
  {
    return index < other.index;
  }
  bool operator==( const IndexPoint& other ) const
  {
    return index == other.index;
  }

  int index = 0;
};


class Disk: public Point {
 public:
  Disk(const Point& p): Point(p) {};
  std::set<IndexPoint> points;
  //unsigned long sign = 0;
  bool prune = false;
  unsigned duration = 0;
};

class DiskNode;

class DiskNode: public Disk {
 public:
  DiskNode(const Disk& d, const int _t): Disk(d), t(_t) {
    assert(d.points.size());
  };
  int t;
  int id = -1;

  Point calc_center() const {
    assert(points.size());
    Point center(0,0);
    for (auto &p : points) {
      center.x += p.x;
      center.y += p.y;
    }
    center.x /= points.size();
    center.y /= points.size();
    return center;
  }

  bool present(const int index) const {
    for (auto &p : points) {
      if (p.index == index) {
        return true;
      }
    }
    return false;
  }

  std::vector<DiskNode>::const_iterator it_prev;
};


class DiskStore: public std::vector<DiskNode> {
 public:
  void insert_disk(const Disk& c, const double epsilon_sq, const int t)
  {
    /*
      for (auto d: C) {
      if ((c.sign && d.sign == c.sign) && (dist_sq(c, d) <= epsilon_sq)) {
      if (d == c) {
      return;
      }
      } else if ((c.sign && d.sign == d.sign) && (dist_sq(c, d) <= epsilon_sq)) {
      if (d == c) {
      // TODO remove d;
      }
      }
      }
    */
    assert(c.points.size());
    for (auto &d: *this) {
      if (c.dist_sq(d) <= epsilon_sq) {
        std::set<IndexPoint> s_a;
        std::set_intersection(c.points.begin(), c.points.end(), d.points.begin(), d.points.end(), std::inserter(s_a, s_a.begin()));
        if (s_a == c.points) {
          // all the points in c are in d
          return; // no need to insert
        } else if (s_a == d.points) {
          // all the points in d are in c
          d.prune = true;
        }
      }
    };

    prune_disks();

    push_back(DiskNode(c,t));
  }

  void prune_disks()
  {
    erase(std::remove_if(begin(), end(),
                         [](auto x){ return x.prune;}), end());
  }
};


class BoundingBox {
 public:
  BoundingBox(const Point& _ll, const Point& _ur): ll(_ll), ur(_ur) {}
  Point ll;
  Point ur;
  std::list<Disk> disks;

  static BoundingBox from_vector(const std::list<IndexPoint>& v) {
    auto xExtremes = std::minmax_element(v.begin(), v.end(),
                                         [](const Point& lhs, const Point& rhs) {
                                           return lhs.x < rhs.x;
                                         });

    auto yExtremes = std::minmax_element(v.begin(), v.end(),
                                         [](const Point& lhs, const Point& rhs) {
                                           return lhs.y < rhs.y;
                                         });
    return BoundingBox(Point(xExtremes.first->x, yExtremes.first->y),
                       Point(xExtremes.second->x, yExtremes.second->y));
  }

  bool intersects_with(const BoundingBox& other) const
  {
    return !(other.ll.x > ur.x
             || other.ur.x < ll.x
             || other.ur.y < ll.y
             || other.ll.y > ur.y);
  }

};


class PSIAlgorithm {
 public:
  PSIAlgorithm(const double &_epsilon, const unsigned _mu, const unsigned _min_duration):
      epsilon(_epsilon), epsilon_sq(_epsilon*_epsilon), mu(_mu), min_duration(_min_duration) {};

  const double epsilon;
  const double epsilon_sq;
  const unsigned mu;
  const unsigned min_duration;

 public:
  std::list<BoundingBox> find_candidate_disks(const std::list<IndexPoint> &T) const;

 protected:
  std::list<DiskStore> disk_store;

  std::vector<Disk> calc_disks(const Point& p1, const Point& p2) const;
  DiskStore filter_candidates(std::list<BoundingBox> &B, const int t) const;
  void connect_disks(DiskStore &disks, DiskStore& disks_prev) const;
  void mark_all_prune(DiskStore& disks) {
    for_each(disks.begin(), disks.end(), [](DiskNode& d){ d.prune = true; });
  }
};


}
