#include "plane_seg/plane_pose.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Eigenvalues>

namespace plane_seg {
namespace {

using Point2 = Eigen::Vector2f;

float cross(const Point2& o, const Point2& a, const Point2& b) {
  return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

// Andrew's monotone chain; returns the hull in counter-clockwise order.
std::vector<Point2> convexHull(std::vector<Point2> pts) {
  std::sort(pts.begin(), pts.end(), [](const Point2& a, const Point2& b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  if (pts.size() < 3) return pts;

  std::vector<Point2> hull(2 * pts.size());
  std::size_t k = 0;
  for (const auto& p : pts) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0) --k;
    hull[k++] = p;
  }
  for (std::size_t i = pts.size() - 1, lower = k + 1; i-- > 0;) {
    while (k >= lower && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0) --k;
    hull[k++] = pts[i];
  }
  hull.resize(k - 1);
  return hull;
}

struct Rect2 {
  Point2 axis = Point2::UnitX();  // unit direction of the long side
  Point2 center = Point2::Zero();
  Point2 extent = Point2::Zero();  // (long, short)
};

// Minimum-area enclosing rectangle: one side is collinear with a hull edge,
// so testing every edge direction is exact.
Rect2 minAreaRect(const std::vector<Point2>& hull) {
  Rect2 best;
  float best_area = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < hull.size(); ++i) {
    Point2 edge = hull[(i + 1) % hull.size()] - hull[i];
    if (edge.norm() < 1e-9f) continue;
    const Point2 u = edge.normalized();
    const Point2 v(-u.y(), u.x());

    float umin = std::numeric_limits<float>::max(), umax = -umin;
    float vmin = umin, vmax = -umin;
    for (const auto& p : hull) {
      umin = std::min(umin, p.dot(u));
      umax = std::max(umax, p.dot(u));
      vmin = std::min(vmin, p.dot(v));
      vmax = std::max(vmax, p.dot(v));
    }
    const float area = (umax - umin) * (vmax - vmin);
    if (area < best_area) {
      best_area = area;
      best.center = u * 0.5f * (umin + umax) + v * 0.5f * (vmin + vmax);
      if (umax - umin >= vmax - vmin) {
        best.axis = u;
        best.extent = Point2(umax - umin, vmax - vmin);
      } else {
        best.axis = v;
        best.extent = Point2(vmax - vmin, umax - umin);
      }
    }
  }
  return best;
}

// Quantile q of the values (partially sorts a copy).
float quantile(std::vector<float> v, float q) {
  const auto k = static_cast<std::size_t>(q * (v.size() - 1));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k];
}

// Rectangle that is robust to stray points attached to the face.
// 1. Orientation from the min-area rectangle of the hull.
// 2. Extent from the trimmed [a, 1-a] quantiles along each axis. For points
//    spread uniformly over a rectangle the trimmed span is (1-2a) of the side,
//    so it is divided by (1-2a) to stay unbiased.
// 3. Points far outside that box are dropped and the orientation is re-fit.
Rect2 robustRect(const std::vector<Point2>& pts) {
  constexpr float kTrim = 0.01f;
  constexpr float kMargin = 0.01f;  // [m] tolerance when dropping strays

  auto measure = [&](const std::vector<Point2>& in, const Point2& axis) {
    const Point2 perp(-axis.y(), axis.x());
    std::vector<float> a, b;
    a.reserve(in.size());
    b.reserve(in.size());
    for (const auto& p : in) {
      a.push_back(p.dot(axis));
      b.push_back(p.dot(perp));
    }
    const float a0 = quantile(a, kTrim), a1 = quantile(a, 1 - kTrim);
    const float b0 = quantile(b, kTrim), b1 = quantile(b, 1 - kTrim);
    Rect2 r;
    r.axis = axis;
    r.center = axis * 0.5f * (a0 + a1) + perp * 0.5f * (b0 + b1);
    r.extent = Point2(a1 - a0, b1 - b0) / (1 - 2 * kTrim);
    return r;
  };

  Rect2 r = measure(pts, minAreaRect(convexHull(pts)).axis);

  const Point2 perp(-r.axis.y(), r.axis.x());
  std::vector<Point2> kept;
  kept.reserve(pts.size());
  for (const auto& p : pts) {
    const Point2 d = p - r.center;
    if (std::abs(d.dot(r.axis)) <= 0.5f * r.extent.x() + kMargin &&
        std::abs(d.dot(perp)) <= 0.5f * r.extent.y() + kMargin)
      kept.push_back(p);
  }
  if (kept.size() >= 3) r = measure(kept, minAreaRect(convexHull(kept)).axis);

  if (r.extent.x() < r.extent.y()) {
    r.axis = Point2(-r.axis.y(), r.axis.x());
    r.extent = Point2(r.extent.y(), r.extent.x());
  }
  return r;
}

}  // namespace

PlanePose estimatePlanePose(const Cloud& plane, const Eigen::Vector4f& coefficients,
                            const Eigen::Vector3f& viewpoint) {
  PlanePose pose;
  if (plane.empty()) return pose;

  Eigen::Vector3f n = coefficients.head<3>();
  const float scale = n.norm();
  n /= scale;
  float d = coefficients[3] / scale;

  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (const auto& p : plane) centroid += p.getVector3fMap();
  centroid /= static_cast<float>(plane.size());

  if (n.dot(viewpoint - centroid) < 0.0f) {
    n = -n;
    d = -d;
  }

  // In-plane basis: any unit vector orthogonal to n, then n x u.
  const Eigen::Vector3f helper =
      std::abs(n.x()) < 0.9f ? Eigen::Vector3f::UnitX() : Eigen::Vector3f::UnitY();
  const Eigen::Vector3f u = n.cross(helper).normalized();
  const Eigen::Vector3f v = n.cross(u);
  const Eigen::Vector3f origin = centroid - (n.dot(centroid) + d) * n;

  std::vector<Point2> pts;
  pts.reserve(plane.size());
  double sq_err = 0.0;
  for (const auto& p : plane) {
    const Eigen::Vector3f q = p.getVector3fMap();
    const float dist = n.dot(q) + d;
    sq_err += static_cast<double>(dist) * dist;
    const Eigen::Vector3f r = q - dist * n - origin;
    pts.emplace_back(r.dot(u), r.dot(v));
  }

  const Rect2 rect = robustRect(pts);

  Eigen::Vector3f x = (rect.axis.x() * u + rect.axis.y() * v).normalized();
  // Resolve the 180-degree ambiguity of the long axis deterministically.
  if (x.dot(Eigen::Vector3f::UnitX()) < 0.0f) x = -x;
  const Eigen::Vector3f y = n.cross(x);

  pose.normal = n;
  pose.center = origin + rect.center.x() * u + rect.center.y() * v;
  pose.extent = rect.extent;
  pose.rms_error = static_cast<float>(std::sqrt(sq_err / plane.size()));
  pose.transform.linear().col(0) = x;
  pose.transform.linear().col(1) = y;
  pose.transform.linear().col(2) = n;
  pose.transform.translation() = pose.center;
  return pose;
}

}  // namespace plane_seg
