#include "Flock.hpp"
#include <map>

using namespace Flock;

std::vector<Disk> PSIAlgorithm::calc_disks(const Point& p1, const Point& p2) const
{
  std::vector<Disk> disks;
  const double D2 = p1.dist_sq(p2);
  if (D2 == 0) {
    return disks;
  }
  const double root = sqrt(epsilon_sq/D2-1);
  const double X = p1.x - p2.x;
  const double Y = p1.y - p2.y;

  disks.push_back(Point((X + Y * root)/2 + p2.x, (Y - X * root)/2 + p2.y));
  disks.push_back(Point((X - Y * root)/2 + p2.x, (Y + X * root)/2 + p2.y));
  return disks;
}


std::list<BoundingBox> PSIAlgorithm::find_candidate_disks(const std::list<IndexPoint> &T) const
{
  std::list<BoundingBox> candidates;

  for (auto &pr : T) {
    std::list<IndexPoint> P;

    std::copy_if (T.begin(), T.end(), std::back_inserter(P),
                  [this, pr](const IndexPoint &ps){
                    return (std::abs(ps.x-pr.x)<= this->epsilon)
                        && (std::abs(ps.y-pr.y)<= this->epsilon)
                        ;} );

    if (P.size() < mu)
      continue;

    BoundingBox MBR = BoundingBox::from_vector(P);

    for (auto &p : P) {
      if (p.x < pr.x)
        continue;
      const double d_sq = p.dist_sq(pr);
      if (d_sq <= epsilon_sq) {
        auto disks = calc_disks(pr, p);
        for (auto &c : disks) {

          std::copy_if (P.begin(), P.end(),
                        std::inserter(c.points, c.points.end()),
                        [this, c](const IndexPoint &p){ return p.dist_sq(c) <= this->epsilon_sq;} );

          if (c.points.size() >= mu) {
            MBR.disks.push_back(c);
          }
        }
      }
    }

    if (MBR.disks.size()) {
      candidates.push_back(MBR);
    }

  }

  return candidates;
}


DiskStore PSIAlgorithm::filter_candidates(std::list<BoundingBox> &B, const int t) const
{
  DiskStore C;
  if (B.size()==0) {
    return C;
  } else if (B.size()==1) {
    for (auto &c : B.front().disks) {
      C.insert_disk(c, epsilon_sq, t);
    }
    return C;
  }

  for (auto j = B.begin(); j!= B.end(); ++j) {
    BoundingBox& Bj = *j;

    auto k = j;
    while (++k != B.end()) {
      BoundingBox& Bk = *k;

      if (Bj.intersects_with(Bk)) {
        for (auto &c : Bj.disks) {
          C.insert_disk(c, epsilon_sq, t);
        }
      } else {
        break;
      }
    }
  }

  // give each one an id
  int i = 0;
  for (auto &c: C) {
    c.id = i++;
  }

  return C;
}

class InvIndex: public std::map<int, std::set<int>>
{
 public:
  InvIndex(const DiskStore& C)
  {
    int i = 0;
    for (auto &d: C) {
      for (auto &p: d.points) {
        (*this)[p.index].insert(i);
      };
      i++;
    }
  }

  DiskStore find_connected_disks(const Disk& d, DiskStore& disks_prev, const unsigned mu) const
  {
    DiskStore connected;
    std::set<int> total;

    unsigned j = 0;
    for (auto &p : d.points) {
      auto it = find(p.index);
      if (it == end()) {
        continue;
      }
      auto &s = it->second;
      if (j==0) {
        total = s; // first point
      } else {
        std::set<int> new_total;
        std::set_intersection(s.begin(), s.end(), total.begin(), total.end(), std::inserter(new_total, new_total.begin()));
        total = new_total;
      }
      j++;
    }

    // only consider connected if more than mu objects in common
    if (j>= mu) {
      for (auto &i: total) {
        disks_prev[i].prune = false;
        connected.push_back(disks_prev[i]);
      }
    }
    return connected;
  }
};


void PSIAlgorithm::connect_disks(DiskStore &disks, DiskStore& disks_prev) const
{
  if (!disks_prev.size() || !disks.size())
    return;

  const InvIndex inv_index(disks_prev);

  // process each current disk
  for (auto &d : disks) {
    assert(d.points.size());

    // find matching disks in previous frame
    const DiskStore connected_disks_prev = inv_index.find_connected_disks(d, disks_prev, mu);

    if (connected_disks_prev.size()) {
      auto ibest = connected_disks_prev.cbegin();

      for (auto it = connected_disks_prev.cbegin(); it != connected_disks_prev.cend(); ++it) {
        assert(it->points.size());
        if (it->duration < d.duration) {
          continue;
        } else if (it->duration > d.duration) {
          d.duration = std::max(d.duration, it->duration);
        } else if (it->points.size() < ibest->points.size()) {
          continue;
        }
        ibest = it;
      }

      for (auto it = disks_prev.begin(); it != disks_prev.end(); ++it) {
        if (it->id == ibest->id) {
          d.it_prev = it;
          d.duration += d.t - ibest->t;
        }
      }
    }
  }
}
