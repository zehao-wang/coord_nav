// obstacle_circles_node.cpp
//
// Fast lidar clustering into obstacle circles ("collision spheres").
//
// Subscribes /scan, converts each valid beam into a base-frame XY point, and
// clusters them into circles [x, y, r] (metres, base frame). Published on a
// fixed configurable rate off the latest scan. The clustering is a 1:1 port of
// perception_server.py (DBSCAN -> centroid enclosing circle -> long-cluster
// split -> redundancy drop), just in C++ so it runs in single-digit ms instead
// of ~80 ms.
//
// Outputs:
//   /obstacles      std_msgs/Float32MultiArray, data = [x0,y0,r0, x1,y1,r1,...]
//                   metres, base_footprint frame. x forward, y left. THIS is the
//                   feed remote consumers read.
//   /obstacles_viz  visualization_msgs/MarkerArray, one cylinder per circle,
//                   for rviz (Fixed Frame = base_footprint).
//
// Params (~private):
//   scan_topic (/scan), rate_hz (3.0; <=0 => process every new scan),
//   eps (0.20), min_samples (5), margin (0.10), max_radius (0.30),
//   frame_id (base_footprint), lidar_yaw (pi), lidar_x (0), lidar_y (0),
//   range_min (0), range_max (0 => use scan's own).
//
// The lidar_* params must match the base_footprint->laser static transform in
// viz.launch (default there: xyz 0 0 0.15, yaw pi). We do the 2D transform here
// rather than depend on tf, since the mount is fixed.

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

namespace {

struct Pt { float x, y; };
struct Circle { float x, y, r; };

// ---- DBSCAN, matching perception_server.dbscan -------------------------
// Brute-force O(n^2) neighbour search: for a few hundred lidar points this is
// well under a millisecond and needs no KD-tree dependency. Border points get
// a label but do not expand their cluster (identical to the Python version).
std::vector<int> dbscan(const std::vector<Pt>& p, float eps, int min_samples) {
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
Circle enclosing_circle(const std::vector<Pt>& pts) {
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

// perception_server.wrap_cluster: one circle, or split a long cluster into
// chunks each within (max_radius - margin) reach of the chunk start. Point
// order (scan order within the cluster) is preserved, as in Python.
void wrap_cluster(const std::vector<Pt>& pts, float margin, float max_radius,
                  std::vector<Circle>& out) {
  Circle c = enclosing_circle(pts);
  if (c.r + margin <= max_radius) {
    c.r += margin;
    out.push_back(c);
    return;
  }
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
    Circle cc = enclosing_circle(chunk);
    cc.r += margin;
    out.push_back(cc);
    i = j;
  }
}

// perception_server.drop_redundant: greedily remove circles (smallest radius
// first) whose covered points stay covered by the remaining kept circles.
std::vector<Circle> drop_redundant(const std::vector<Circle>& circles,
                                   const std::vector<Pt>& points) {
  const int m = static_cast<int>(circles.size());
  if (m < 2 || points.empty()) return circles;
  const int np = static_cast<int>(points.size());

  // covers[i][p] : circle i covers point p
  std::vector<std::vector<char>> covers(m, std::vector<char>(np, 0));
  for (int i = 0; i < m; ++i) {
    const float rr = circles[i].r + 1e-9f;
    const float r2 = rr * rr;
    for (int p = 0; p < np; ++p) {
      const float dx = points[p].x - circles[i].x, dy = points[p].y - circles[i].y;
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
      if (!covers[idx][p]) continue;        // only points this circle covers
      bool covered = false;
      for (int k = 0; k < m; ++k) {
        if (keep[k] && covers[k][p]) { covered = true; break; }
      }
      if (!covered) redundant = false;      // dropping idx would uncover p
    }
    if (!redundant) keep[idx] = 1;          // put it back
  }

  std::vector<Circle> out;
  for (int i = 0; i < m; ++i)
    if (keep[i]) out.push_back(circles[i]);
  return out;
}

}  // namespace

class ObstacleCirclesNode {
 public:
  ObstacleCirclesNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("scan_topic", scan_topic_, "/scan");
    pnh.param<std::string>("frame_id", frame_id_, "base_footprint");
    pnh.param("rate_hz", rate_hz_, 3.0);
    pnh.param("eps", eps_, 0.20);
    pnh.param("min_samples", min_samples_, 5);
    pnh.param("margin", margin_, 0.10);
    pnh.param("max_radius", max_radius_, 0.30);
    pnh.param("lidar_yaw", lidar_yaw_, M_PI);
    pnh.param("lidar_x", lidar_x_, 0.0);
    pnh.param("lidar_y", lidar_y_, 0.0);
    pnh.param("range_min", range_min_, 0.0);
    pnh.param("range_max", range_max_, 0.0);

    cyaw_ = std::cos(lidar_yaw_);
    syaw_ = std::sin(lidar_yaw_);

    pub_ = nh.advertise<std_msgs::Float32MultiArray>("obstacles", 5);
    pub_viz_ = nh.advertise<visualization_msgs::MarkerArray>("obstacles_viz", 5);
    sub_ = nh.subscribe(scan_topic_, 1, &ObstacleCirclesNode::onScan, this);

    if (rate_hz_ > 0.0) {
      timer_ = nh.createTimer(ros::Duration(1.0 / rate_hz_),
                              &ObstacleCirclesNode::onTimer, this);
      ROS_INFO("obstacle_circles: timer at %.2f Hz off latest scan", rate_hz_);
    } else {
      ROS_INFO("obstacle_circles: process every new scan (max rate)");
    }
    ROS_INFO("params eps=%.2f min_samples=%d margin=%.2f max_radius=%.2f",
             eps_, min_samples_, margin_, max_radius_);
  }

 private:
  void onScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    last_scan_ = msg;
    if (rate_hz_ <= 0.0) process();          // event-driven max-rate mode
  }

  void onTimer(const ros::TimerEvent&) { process(); }

  void process() {
    if (!last_scan_) return;
    if (last_scan_->header.stamp == last_done_stamp_) return;  // no new scan
    last_done_stamp_ = last_scan_->header.stamp;

    const ros::WallTime t0 = ros::WallTime::now();
    const sensor_msgs::LaserScan& s = *last_scan_;

    const float rmin = (range_min_ > 0.0) ? range_min_ : s.range_min;
    const float rmax = (range_max_ > 0.0) ? range_max_ : s.range_max;

    // beams -> base-frame points
    std::vector<Pt> pts;
    pts.reserve(s.ranges.size());
    for (size_t i = 0; i < s.ranges.size(); ++i) {
      const float r = s.ranges[i];
      if (!std::isfinite(r) || r <= 0.0f || r < rmin || r > rmax) continue;
      const float a = s.angle_min + i * s.angle_increment;
      const float lx = r * std::cos(a), ly = r * std::sin(a);
      Pt bp;
      bp.x = static_cast<float>(cyaw_ * lx - syaw_ * ly + lidar_x_);
      bp.y = static_cast<float>(syaw_ * lx + cyaw_ * ly + lidar_y_);
      pts.push_back(bp);
    }

    // cluster -> circles
    std::vector<Circle> circles;
    std::vector<Pt> cluster_pts;
    if (!pts.empty()) {
      std::vector<int> labels = dbscan(pts, eps_, min_samples_);
      int max_lab = -1;
      for (int l : labels) max_lab = std::max(max_lab, l);
      for (int lab = 0; lab <= max_lab; ++lab) {
        std::vector<Pt> cluster;
        for (size_t i = 0; i < pts.size(); ++i)
          if (labels[i] == lab) cluster.push_back(pts[i]);
        if (cluster.empty()) continue;
        for (const Pt& q : cluster) cluster_pts.push_back(q);
        wrap_cluster(cluster, margin_, max_radius_, circles);
      }
      circles = drop_redundant(circles, cluster_pts);
    }

    const double ms = (ros::WallTime::now() - t0).toSec() * 1000.0;

    ++frame_counter_;                 // this is the sample id ("frame_id")
    publish(circles, s.header.stamp);
    publishViz(circles, s.header.stamp);

    // rolling stats so we can measure the fastest stable rate
    ema_ms_ = (ema_ms_ < 0) ? ms : 0.9 * ema_ms_ + 0.1 * ms;
    ++frames_;
    const ros::WallTime now = ros::WallTime::now();
    if ((now - last_report_).toSec() >= 2.0) {
      const double hz = frames_ / (now - last_report_).toSec();
      ROS_INFO("circles=%zu  proc=%.2f ms (ema)  pub=%.2f Hz",
               circles.size(), ema_ms_, hz);
      frames_ = 0;
      last_report_ = now;
    }
  }

  void publish(const std::vector<Circle>& circles, const ros::Time&) {
    // data = [frame_id, x0,y0,r0, x1,y1,r1, ...]
    //   frame_id : monotonic sample counter (data[0])
    //   then 3 floats per circle: x forward, y left, r radius (metres, base frame)
    std_msgs::Float32MultiArray m;
    m.layout.data_offset = 1;            // circle data begins after frame_id
    m.layout.dim.resize(1);
    m.layout.dim[0].label = "frame_id;xyr_triples";
    m.layout.dim[0].size = circles.size();
    m.layout.dim[0].stride = circles.size() * 3;
    m.data.reserve(1 + circles.size() * 3);
    m.data.push_back(static_cast<float>(frame_counter_));
    for (const Circle& c : circles) {
      m.data.push_back(c.x);
      m.data.push_back(c.y);
      m.data.push_back(c.r);
    }
    pub_.publish(m);
  }

  void publishViz(const std::vector<Circle>& circles, const ros::Time& stamp) {
    if (pub_viz_.getNumSubscribers() == 0) return;
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.header.frame_id = frame_id_;
    del.header.stamp = stamp;
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);
    int id = 0;
    for (const Circle& c : circles) {
      visualization_msgs::Marker mk;
      mk.header.frame_id = frame_id_;
      mk.header.stamp = stamp;
      mk.ns = "obstacles";
      mk.id = id++;
      mk.type = visualization_msgs::Marker::CYLINDER;
      mk.action = visualization_msgs::Marker::ADD;
      mk.pose.position.x = c.x;
      mk.pose.position.y = c.y;
      mk.pose.position.z = 0.0;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = 2.0 * c.r;
      mk.scale.y = 2.0 * c.r;
      mk.scale.z = 0.05;
      mk.color.r = 1.0f; mk.color.g = 0.3f; mk.color.b = 0.1f; mk.color.a = 0.5f;
      mk.lifetime = ros::Duration(0.5);
      arr.markers.push_back(mk);
    }
    pub_viz_.publish(arr);
  }

  ros::Subscriber sub_;
  ros::Publisher pub_, pub_viz_;
  ros::Timer timer_;
  sensor_msgs::LaserScan::ConstPtr last_scan_;
  ros::Time last_done_stamp_;

  std::string scan_topic_, frame_id_;
  double rate_hz_, eps_, margin_, max_radius_;
  int min_samples_;
  double lidar_yaw_, lidar_x_, lidar_y_, range_min_, range_max_;
  double cyaw_, syaw_;

  double ema_ms_ = -1.0;
  int frames_ = 0;
  unsigned long frame_counter_ = 0;   // monotonic sample id (the "frame_id")
  ros::WallTime last_report_ = ros::WallTime::now();
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_circles");
  ros::NodeHandle nh, pnh("~");
  ObstacleCirclesNode node(nh, pnh);
  ros::spin();
  return 0;
}
