// clustering.h -- the PURE lidar->circles extraction logic, ROS-free.
//
// Split out of obstacle_circles_node.cpp so the exact code the car runs can be
// unit-tested on the workstation (tests/test_clustering.cpp, plain g++). The
// node keeps only the ROS plumbing + the ego-motion temporal filter.
//
// 2026-08-05 extraction fixes (each mapped to a pathology MEASURED by replaying
// 220 recorded on-car ticks through the planner's tracker -- CHANGELOG 0.9.20):
//
//  * kasa_fit + stable_circle: the old centroid+farthest-point circle put a
//    small object's centre on its VISIBLE FACE, so the centre slid toward the
//    car as it approached (a phantom radial velocity for the tracker). For a
//    compact arc-like cluster, an algebraic (Kasa) circle fit recovers the
//    true centre BEHIND the face, viewpoint-independent. Gated: only used
//    when the fitted radius is credible (<= max_radius) and the residual is
//    small; walls/lines fall back to the centroid circle.
//  * wrap_cluster_grid: the old long-cluster split walked scan order breaking
//    a chunk where it out-ran `reach` FROM THE CHUNK START -- the whole
//    boundary chain slid along a wall as the viewpoint moved (the dominant
//    phantom-velocity source: wall centroids creeping at ~0.1-0.3 m/s). With
//    odom available, chunk boundaries now come from a FIXED ODOM-FRAME GRID:
//    the car moves, the boundaries do not. Falls back to the old walk without
//    odom.
//  * merge_circles: clustering sometimes returned ONE object as TWO circles
//    (thin mid-section, split threshold); nothing ever merged them back, and
//    the pair destabilised the tracker. Circles whose union still fits in
//    max_radius are now merged (repeat to fixpoint).

#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

namespace obstacle_clustering {

struct Pt { float x, y; };
struct Circle { float x, y, r; };

// ---- DBSCAN, matching perception_server.dbscan -------------------------
// Brute-force O(n^2) neighbour search: for a few hundred lidar points this is
// well under a millisecond and needs no KD-tree dependency. Border points get
// a label but do not expand their cluster (identical to the Python version).
inline std::vector<int> dbscan(const std::vector<Pt>& p, float eps,
                               int min_samples) {
  const int n = static_cast<int>(p.size());
  std::vector<int> labels(n, -1);
  if (n == 0) return labels;

  const float eps2 = eps * eps;
  std::vector<std::vector<int>> nbr(n);
  for (int i = 0; i < n; ++i) {
    nbr[i].push_back(i);  // include self, as the Python neighbour count does
    for (int j = i + 1; j < n; ++j) {
      const float dx = p[i].x - p[j].x, dy = p[i].y - p[j].y;
      if (dx * dx + dy * dy <= eps2) {
        nbr[i].push_back(j);
        nbr[j].push_back(i);
      }
    }
  }

  std::vector<char> core(n), visited(n, 0);
  for (int i = 0; i < n; ++i)
    core[i] = static_cast<int>(nbr[i].size()) >= min_samples;

  int cid = -1;
  std::vector<int> stack;
  for (int i = 0; i < n; ++i) {
    if (visited[i] || !core[i]) continue;
    ++cid;
    stack.clear();
    stack.push_back(i);
    visited[i] = 1;
    labels[i] = cid;
    while (!stack.empty()) {
      const int q = stack.back();
      stack.pop_back();
      if (!core[q]) continue;              // border points do not expand
      for (int k : nbr[q]) {
        if (!visited[k]) {
          visited[k] = 1;
          labels[k] = cid;
          stack.push_back(k);
        } else if (labels[k] == -1) {
          labels[k] = cid;                 // reclaim a point marked noise
        }
      }
    }
  }
  return labels;
}

// Centroid + farthest-point radius (perception_server.enclosing_circle).
inline Circle enclosing_circle(const std::vector<Pt>& pts) {
  Circle c{0, 0, 0};
  const int n = static_cast<int>(pts.size());
  if (n == 0) return c;
  double sx = 0, sy = 0;
  for (const Pt& q : pts) { sx += q.x; sy += q.y; }
  c.x = static_cast<float>(sx / n);
  c.y = static_cast<float>(sy / n);
  float r2 = 0;
  for (const Pt& q : pts) {
    const float dx = q.x - c.x, dy = q.y - c.y;
    r2 = std::max(r2, dx * dx + dy * dy);
  }
  c.r = (n > 1) ? std::sqrt(r2) : 0.0f;
  return c;
}

// Algebraic (Kasa) circle fit: minimise sum((x^2+y^2) + D x + E y + F)^2.
// Returns false when the 3x3 normal system is near-singular (collinear points
// -- a wall) or the fit is not credible. rms_out = RMS radial residual.
inline bool kasa_fit(const std::vector<Pt>& pts, Circle& out, float& rms_out) {
  const int n = static_cast<int>(pts.size());
  if (n < 3) return false;
  // normal equations for [D E F]
  double a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = n;
  double b1 = 0, b2 = 0, b3 = 0;
  for (const Pt& q : pts) {
    const double x = q.x, y = q.y, z = x * x + y * y;
    a11 += x * x; a12 += x * y; a13 += x;
    a22 += y * y; a23 += y;
    b1 -= z * x; b2 -= z * y; b3 -= z;
  }
  // solve symmetric 3x3 by elimination
  double m[3][4] = {{a11, a12, a13, b1}, {a12, a22, a23, b2}, {a13, a23, a33, b3}};
  for (int c = 0; c < 3; ++c) {
    int piv = c;
    for (int r = c + 1; r < 3; ++r)
      if (std::fabs(m[r][c]) > std::fabs(m[piv][c])) piv = r;
    if (std::fabs(m[piv][c]) < 1e-9) return false;
    if (piv != c) for (int k = c; k < 4; ++k) std::swap(m[piv][k], m[c][k]);
    for (int r = 0; r < 3; ++r) {
      if (r == c) continue;
      const double f = m[r][c] / m[c][c];
      for (int k = c; k < 4; ++k) m[r][k] -= f * m[c][k];
    }
  }
  const double D = m[0][3] / m[0][0], E = m[1][3] / m[1][1], F = m[2][3] / m[2][2];
  const double cx = -D / 2.0, cy = -E / 2.0;
  const double r2 = cx * cx + cy * cy - F;
  if (!(r2 > 0.0) || !std::isfinite(r2)) return false;
  const double r = std::sqrt(r2);
  double ss = 0;
  for (const Pt& q : pts) {
    const double d = std::hypot(q.x - cx, q.y - cy) - r;
    ss += d * d;
  }
  out.x = static_cast<float>(cx);
  out.y = static_cast<float>(cy);
  out.r = static_cast<float>(r);
  rms_out = static_cast<float>(std::sqrt(ss / n));
  return true;
}

// The circle for a COMPACT cluster: Kasa fit when it is credible (the visible
// arc of a small round-ish object -> true centre behind the face, stable under
// viewpoint change), else the centroid circle. kasa_rms is the residual gate
// (m): larger = accept sloppier arcs.
inline Circle stable_circle(const std::vector<Pt>& pts, float margin,
                            float max_radius, float kasa_rms) {
  Circle cen = enclosing_circle(pts);
  Circle fit;
  float rms;
  if (kasa_fit(pts, fit, rms) && rms <= kasa_rms &&
      fit.r + margin <= max_radius && fit.r >= 0.5f * cen.r) {
    fit.r += margin;
    return fit;
  }
  cen.r += margin;
  return cen;
}

// Old long-cluster split (kept as the no-odom fallback): walk scan order and
// break a chunk where a point out-runs `reach` of the CHUNK START. NOTE: the
// boundaries slide with the viewpoint; prefer wrap_cluster_grid.
inline void wrap_cluster_walk(const std::vector<Pt>& pts, float margin,
                              float max_radius, float kasa_rms,
                              std::vector<Circle>& out) {
  const float reach = std::max(max_radius - margin, 1e-3f);
  const int n = static_cast<int>(pts.size());
  int i = 0;
  while (i < n) {
    int j = i + 1;
    while (j < n) {
      const float dx = pts[j].x - pts[i].x, dy = pts[j].y - pts[i].y;
      if (std::sqrt(dx * dx + dy * dy) > reach) break;
      ++j;
    }
    std::vector<Pt> chunk(pts.begin() + i, pts.begin() + j);
    out.push_back(stable_circle(chunk, margin, max_radius, kasa_rms));
    i = j;
  }
}

// Long-cluster split with ODOM-ANCHORED boundaries: quantise each point's odom
// position to a fixed grid of `cell` metres and emit one circle per occupied
// cell (base-frame points, so downstream stays unchanged). The car moves, the
// cell boundaries do not -- wall chunk centroids stop sliding. A cell whose
// chunk still exceeds max_radius (diagonal fill) falls back to the walk within
// that chunk. odom = (car_x, car_y, car_yaw).
inline void wrap_cluster_grid(const std::vector<Pt>& pts, float margin,
                              float max_radius, float kasa_rms, float cell,
                              const double odom[3], std::vector<Circle>& out) {
  const double c = std::cos(odom[2]), s = std::sin(odom[2]);
  // cell key per point; group scan-order runs of the same key first, then
  // gather split runs of the same cell (rare) via a flat pass
  const int n = static_cast<int>(pts.size());
  std::vector<long long> key(n);
  for (int i = 0; i < n; ++i) {
    const double ox = odom[0] + c * pts[i].x - s * pts[i].y;
    const double oy = odom[1] + s * pts[i].x + c * pts[i].y;
    const long long kx = static_cast<long long>(std::floor(ox / cell));
    const long long ky = static_cast<long long>(std::floor(oy / cell));
    key[i] = (kx << 32) ^ (ky & 0xffffffffLL);
  }
  std::vector<char> done(n, 0);
  for (int i = 0; i < n; ++i) {
    if (done[i]) continue;
    std::vector<Pt> chunk;
    for (int j = i; j < n; ++j) {
      if (!done[j] && key[j] == key[i]) {
        chunk.push_back(pts[j]);
        done[j] = 1;
      }
    }
    Circle cc = enclosing_circle(chunk);
    if (cc.r + margin <= max_radius) {
      out.push_back(stable_circle(chunk, margin, max_radius, kasa_rms));
    } else {
      wrap_cluster_walk(chunk, margin, max_radius, kasa_rms, out);
    }
  }
}

// perception_server.wrap_cluster, upgraded: one stable circle when compact;
// long clusters split by odom grid (or the walk when odom is unavailable).
inline void wrap_cluster(const std::vector<Pt>& pts, float margin,
                         float max_radius, float kasa_rms, float grid_cell,
                         const double* odom,   // nullptr = no odom
                         std::vector<Circle>& out) {
  Circle c = enclosing_circle(pts);
  if (c.r + margin <= max_radius) {
    out.push_back(stable_circle(pts, margin, max_radius, kasa_rms));
    return;
  }
  if (odom != nullptr)
    wrap_cluster_grid(pts, margin, max_radius, kasa_rms, grid_cell, odom, out);
  else
    wrap_cluster_walk(pts, margin, max_radius, kasa_rms, out);
}

// Merge circles whose UNION still fits within max_radius (a split object comes
// back as one circle; adjacent tiny wall chunks coalesce). Runs to fixpoint;
// O(m^2) per pass on a handful of circles.
inline std::vector<Circle> merge_circles(std::vector<Circle> cs,
                                         float max_radius) {
  bool merged = true;
  while (merged) {
    merged = false;
    for (size_t i = 0; i < cs.size() && !merged; ++i) {
      for (size_t j = i + 1; j < cs.size() && !merged; ++j) {
        const float dx = cs[j].x - cs[i].x, dy = cs[j].y - cs[i].y;
        const float d = std::hypot(dx, dy);
        // smallest circle containing both
        float ru = 0.5f * (d + cs[i].r + cs[j].r);
        if (ru < cs[i].r) ru = cs[i].r;       // one contains the other
        if (ru < cs[j].r) ru = cs[j].r;
        if (ru > max_radius) continue;
        Circle u;
        if (d < 1e-6f) {
          u = (cs[i].r >= cs[j].r) ? cs[i] : cs[j];
        } else {
          // centre on the segment, placed so both circles are enclosed
          const float t = (ru - cs[i].r) / d;
          u.x = cs[i].x + t * dx;
          u.y = cs[i].y + t * dy;
          u.r = ru;
        }
        cs[i] = u;
        cs.erase(cs.begin() + j);
        merged = true;
      }
    }
  }
  return cs;
}

// perception_server.drop_redundant: greedily remove circles (smallest radius
// first) whose covered points stay covered by the remaining kept circles.
inline std::vector<Circle> drop_redundant(const std::vector<Circle>& circles,
                                          const std::vector<Pt>& points) {
  const int m = static_cast<int>(circles.size());
  if (m < 2 || points.empty()) return circles;
  const int np = static_cast<int>(points.size());

  std::vector<std::vector<char>> covers(m, std::vector<char>(np, 0));
  for (int i = 0; i < m; ++i) {
    const float rr = circles[i].r + 1e-9f;
    const float r2 = rr * rr;
    for (int p = 0; p < np; ++p) {
      const float dx = points[p].x - circles[i].x,
                  dy = points[p].y - circles[i].y;
      covers[i][p] = (dx * dx + dy * dy) <= r2;
    }
  }

  std::vector<int> order(m);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](int a, int b) { return circles[a].r < circles[b].r; });

  std::vector<char> keep(m, 1);
  for (int idx : order) {
    keep[idx] = 0;                          // tentatively drop
    bool redundant = true;
    for (int p = 0; p < np && redundant; ++p) {
      if (!covers[idx][p]) continue;
      bool covered = false;
      for (int k = 0; k < m; ++k) {
        if (keep[k] && covers[k][p]) { covered = true; break; }
      }
      if (!covered) redundant = false;
    }
    if (!redundant) keep[idx] = 1;
  }

  std::vector<Circle> out;
  for (int i = 0; i < m; ++i)
    if (keep[i]) out.push_back(circles[i]);
  return out;
}

}  // namespace obstacle_clustering
