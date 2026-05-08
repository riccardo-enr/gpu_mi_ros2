/*
 * octomap_to_grid_node
 *
 * Subscribes to an `octomap_msgs/Octomap` topic, deserializes the octree,
 * computes the tight bounding box of its known leaves, voxelizes onto a dense
 * `(width, height, depth)` grid at the octree resolution, and publishes an
 * `octomap_to_grid/OccupancyGrid3D` message.
 *
 * Conventions
 *   - voxel (i, j, k) center at world coords
 *       (origin.x + (i+0.5)*res, origin.y + (j+0.5)*res, origin.z + (k+0.5)*res)
 *   - data flat layout: index = i*H*D + j*D + k  (k fastest, then j, then i)
 *   - occupied leaf  -> 1.0
 *   - free leaf      -> 0.0
 *   - unknown voxel  -> `occ_prior` parameter (default 0.5)
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

#include "octomap_to_grid/msg/occupancy_grid3_d.hpp"

class OctomapToGridNode : public rclcpp::Node {
public:
  OctomapToGridNode() : Node("octomap_to_grid_node") {
    occ_prior_ = static_cast<float>(declare_parameter<double>("occ_prior", 0.5));
    free_thresh_ = static_cast<float>(declare_parameter<double>("free_thresh", 0.5));

    pub_ = create_publisher<octomap_to_grid::msg::OccupancyGrid3D>("~/grid_out", 1);
    sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        "~/octomap_in", 1,
        std::bind(&OctomapToGridNode::octomapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "octomap_to_grid_node ready  occ_prior=%.2f  free_thresh=%.2f",
                occ_prior_, free_thresh_);
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(
        msg->binary ? octomap_msgs::binaryMsgToMap(*msg)
                    : octomap_msgs::fullMsgToMap(*msg));
    if (!abstract_tree) {
      RCLCPP_WARN(get_logger(), "Failed to deserialize OctoMap message");
      return;
    }

    auto* tree = dynamic_cast<octomap::OcTree*>(abstract_tree.get());
    if (!tree) {
      RCLCPP_WARN(get_logger(), "Deserialized tree is not an OcTree");
      return;
    }

    const double res = tree->getResolution();
    if (res <= 0.0) {
      RCLCPP_WARN(get_logger(), "Non-positive octree resolution: %.6f", res);
      return;
    }

    /*
     * Pass 1: find the tight bbox of known leaves.
     * We snap to a global grid aligned to the octree origin (multiples of res)
     * so the dense grid is consistent across publications.
     */
    bool have_any = false;
    double xmin = 0, ymin = 0, zmin = 0;
    double xmax = 0, ymax = 0, zmax = 0;
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
      const double x = it.getX();
      const double y = it.getY();
      const double z = it.getZ();
      if (!have_any) {
        xmin = xmax = x;
        ymin = ymax = y;
        zmin = zmax = z;
        have_any = true;
      } else {
        xmin = std::min(xmin, x); xmax = std::max(xmax, x);
        ymin = std::min(ymin, y); ymax = std::max(ymax, y);
        zmin = std::min(zmin, z); zmax = std::max(zmax, z);
      }
    }

    octomap_to_grid::msg::OccupancyGrid3D out;
    out.header = msg->header;
    out.resolution = static_cast<float>(res);

    if (!have_any) {
      out.origin.x = 0.0;
      out.origin.y = 0.0;
      out.origin.z = 0.0;
      out.width = 0;
      out.height = 0;
      out.depth = 0;
      out.data.clear();
      pub_->publish(out);
      return;
    }

    /*
     * Treat each leaf center as the center of a voxel of size `res` in the
     * dense grid (we only handle leaves that actually have size == res; coarser
     * inner-node leaves are expanded by sampling each voxel inside their
     * bbox via tree->search).
     */
    const double inv_res = 1.0 / res;
    const auto floor_idx = [&](double v_min) {
      return static_cast<int64_t>(std::floor(v_min * inv_res));
    };
    const auto ceil_idx = [&](double v_max) {
      return static_cast<int64_t>(std::floor(v_max * inv_res));
    };

    const int64_t i0 = floor_idx(xmin);
    const int64_t j0 = floor_idx(ymin);
    const int64_t k0 = floor_idx(zmin);
    const int64_t i1 = ceil_idx(xmax);
    const int64_t j1 = ceil_idx(ymax);
    const int64_t k1 = ceil_idx(zmax);

    const uint32_t W = static_cast<uint32_t>(i1 - i0 + 1);
    const uint32_t H = static_cast<uint32_t>(j1 - j0 + 1);
    const uint32_t D = static_cast<uint32_t>(k1 - k0 + 1);

    out.origin.x = i0 * res;
    out.origin.y = j0 * res;
    out.origin.z = k0 * res;
    out.width = W;
    out.height = H;
    out.depth = D;
    out.data.assign(static_cast<size_t>(W) * H * D, occ_prior_);

    /*
     * Pass 2: fill known voxels by sampling tree->search at every voxel center.
     * This handles inner-node leaves (size > res) by treating each contained
     * voxel as known with the leaf's occupancy probability, and is robust to
     * pruned trees.
     */
    size_t known = 0;
    for (uint32_t i = 0; i < W; ++i) {
      const double x = out.origin.x + (i + 0.5) * res;
      for (uint32_t j = 0; j < H; ++j) {
        const double y = out.origin.y + (j + 0.5) * res;
        for (uint32_t k = 0; k < D; ++k) {
          const double z = out.origin.z + (k + 0.5) * res;
          octomap::OcTreeNode* n = tree->search(x, y, z);
          if (!n) continue;
          const float p = static_cast<float>(n->getOccupancy());
          // Snap to {0, 1} for clear free / occupied; keep the probability for
          // ambiguous mid-range nodes so the MI computation can use it.
          float v;
          if (p >= tree->getOccupancyThres()) {
            v = 1.0f;
          } else if (p < free_thresh_) {
            v = 0.0f;
          } else {
            v = p;
          }
          const size_t idx = (static_cast<size_t>(i) * H + j) * D + k;
          out.data[idx] = v;
          ++known;
        }
      }
    }

    RCLCPP_DEBUG(get_logger(),
                 "OctoMap -> grid: WxHxD=%ux%ux%u  res=%.3f  known=%zu/%zu  origin=(%.2f,%.2f,%.2f)",
                 W, H, D, res, known, out.data.size(),
                 out.origin.x, out.origin.y, out.origin.z);

    pub_->publish(out);
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
  rclcpp::Publisher<octomap_to_grid::msg::OccupancyGrid3D>::SharedPtr pub_;
  float occ_prior_{0.5f};
  float free_thresh_{0.5f};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapToGridNode>());
  rclcpp::shutdown();
  return 0;
}
