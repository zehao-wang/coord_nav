// Workstation unit test for the ROS-free extraction logic (clustering.h).
// Build & run (no ROS needed):
//   g++ -O2 -std=c++14 -I../src tests/test_clustering.cpp -o /tmp/test_clustering && /tmp/test_clustering
//
// Synthesises lidar views of a wall and a small box from a MOVING viewpoint and
// checks the three measured extraction pathologies stay fixed:
//   1. wall chunk centroids do not slide as the car advances (odom grid);
//   2. a box returned as a split cluster comes back as ONE circle (merge);
//   3. the Kasa fit keeps a box's centre put while the car approaches
//      (the centroid circle slid toward the car with the visible arc).

#include <cassert>
#include <cstdio>
#include <cmath>
#include <map>
#include "../src/clustering.h"

using namespace obstacle_clustering;

namespace {

// points of a wall segment y in [y0,y1] at x=wx (odom), seen from a car at
// (0, cy) with a finite sensing range -- the visible WINDOW slides with the
// car, which is what makes walk-chunking boundaries slide too
std::vector<Pt> wall_points(double wx, double y0, double y1, double cy,
                            double range = 2.0) {
  std::vector<Pt> out;
  for (double y = y0; y <= y1; y += 0.03) {
    if (std::hypot(wx, y - cy) > range) continue;
    out.push_back({static_cast<float>(wx), static_cast<float>(y - cy)});
  }
  return out;
}

// visible front arc of a circle (radius R at odom (bx,by)) from car at (cx,0)
std::vector<Pt> box_arc(double bx, double by, double R, double cx) {
  std::vector<Pt> out;
  const double view = std::atan2(by, bx - cx) + M_PI;   // arc faces the car
  for (double a = -1.2; a <= 1.2; a += 0.08) {
    const double px = bx + R * std::cos(view + a), py = by + R * std::sin(view + a);
    out.push_back({static_cast<float>(px - cx), static_cast<float>(py)});
  }
  return out;
}

double dist(const Circle& a, const Circle& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace

int main() {
  const float margin = 0.02f, max_r = 0.30f, kasa = 0.02f, cell = 0.35f;

  // 1. WALL SLIDE: cluster a long wall while the car DRIVES ALONG it (the
  // finite sensing range slides the visible window, which slid every
  // walk-chunk boundary). Odom-grid chunking must keep interior chunk
  // centroids pinned in ODOM; the walk slides them.
  {
    double worst_grid = 0, worst_walk = 0;
    std::vector<Circle> prev_g, prev_w;
    for (int step = 0; step < 2; ++step) {
      const double cy = 0.10 * step;      // one tick of driving along the wall
      auto pts = wall_points(1.5, -4.0, 4.0, cy);
      std::vector<Circle> g, w;
      const double odom[3] = {0.0, cy, 0.0};
      wrap_cluster(pts, margin, max_r, kasa, cell, odom, g);
      wrap_cluster_walk(pts, margin, max_r, kasa, w);
      for (Circle& c : g) c.y += cy;      // base -> odom for comparison
      for (Circle& c : w) c.y += cy;
      if (step) {
        // interior chunks only (the window EDGES legitimately move with the
        // car; the pathology is interior chunks moving too)
        auto worst_of = [&](const std::vector<Circle>& cur,
                            const std::vector<Circle>& prev) {
          double worst = 0;
          for (const Circle& c : cur) {
            if (std::fabs(c.y - cy) > 1.0) continue;   // interior of the view
            double best = 1e9;
            for (const Circle& p : prev) best = std::min(best, dist(c, p));
            worst = std::max(worst, best);
          }
          return worst;
        };
        worst_grid = worst_of(g, prev_g);
        worst_walk = worst_of(w, prev_w);
      }
      prev_g = g;
      prev_w = w;
    }
    std::printf("interior wall-chunk slide per tick: grid %.4f m  walk %.4f m\n",
                worst_grid, worst_walk);
    assert(worst_grid < 0.03);            // odom grid: interior residual = boundary-point
    // quantisation only (a point on a cell line may flip cells)
    assert(worst_walk > 2.0 * worst_grid); // and the walk is clearly worse
  }

  // 2. SPLIT MERGE: two half-arcs of one 0.12 m box, clustered separately
  // (simulating a DBSCAN split), must come back as ONE circle.
  {
    auto arc = box_arc(1.5, 0.0, 0.12, 0.0);
    std::vector<Pt> left(arc.begin(), arc.begin() + arc.size() / 2 - 1);
    std::vector<Pt> right(arc.begin() + arc.size() / 2 + 1, arc.end());
    std::vector<Circle> cs;
    wrap_cluster(left, margin, max_r, kasa, cell, nullptr, cs);
    wrap_cluster(right, margin, max_r, kasa, cell, nullptr, cs);
    assert(cs.size() == 2);
    auto merged = merge_circles(cs, max_r);
    std::printf("split box: %zu circles -> %zu after merge\n", cs.size(),
                merged.size());
    assert(merged.size() == 1);
    assert(std::fabs(merged[0].x - 1.5) < 0.15);
  }

  // 3. APPROACH STABILITY: the box's circle centre must not creep toward the
  // car as it approaches (centroid-of-arc did; the Kasa fit recovers the true
  // centre). Compare centre estimates from 1.5 m and 0.9 m away, in ODOM.
  {
    Circle far_c, near_c;
    for (int step = 0; step < 2; ++step) {
      const double cx = step ? 0.6 : 0.0;
      auto pts = box_arc(1.5, 0.0, 0.12, cx);
      std::vector<Circle> cs;
      wrap_cluster(pts, margin, max_r, kasa, cell, nullptr, cs);
      assert(cs.size() == 1);
      cs[0].x += cx;
      (step ? near_c : far_c) = cs[0];
    }
    const double drift = dist(far_c, near_c);
    std::printf("box centre drift over 0.6 m approach: %.4f m (centre %.3f,%.3f)\n",
                drift, near_c.x, near_c.y);
    assert(drift < 0.02);                 // was ~0.05-0.10 with the centroid
    assert(std::fabs(near_c.x - 1.5) < 0.05);  // and it is the TRUE centre
  }

  std::printf("all clustering tests passed\n");
  return 0;
}
