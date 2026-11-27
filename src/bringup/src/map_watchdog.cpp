#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
std::atomic<bool> g_stop{false};

void onSignal(int) {g_stop = true;}
}

class MapWatchdog : public rclcpp::Node
{
public:
  MapWatchdog()
  : rclcpp::Node("map_watchdog")
  {
    map2d_topic_ = declare_parameter<std::string>("map2d_topic", "/projected_map");
    map3d_topic_ = declare_parameter<std::string>("map3d_topic", "/octomap_binary");
    out_dir_ = declare_parameter<std::string>("out_dir", "PCD");
    prefix_ = declare_parameter<std::string>("prefix", "map");
    occupied_thresh_ = declare_parameter<double>("occupied_thresh", 0.65);
    free_thresh_ = declare_parameter<double>("free_thresh", 0.196);
    negate_ = declare_parameter<int>("negate", 0);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();
    sub2d_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map2d_topic_, qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {map2d_ = msg;});
    sub3d_ = create_subscription<octomap_msgs::msg::Octomap>(
      map3d_topic_, qos,
      [this](octomap_msgs::msg::Octomap::SharedPtr msg) {map3d_ = msg;});
  }

  void save()
  {
    std::error_code error;
    std::filesystem::create_directories(out_dir_, error);
    if (error) {
      RCLCPP_ERROR(get_logger(), "cannot create %s: %s", out_dir_.c_str(),
                   error.message().c_str());
      return;
    }
    const std::string stamp = nowStamp();
    const std::string base = prefix_ + "_" + stamp;
    std::string written;
    if (map2d_ && save2d(base)) {
      written += base + ".pgm ";
    }
    if (map3d_ && save3d(base)) {
      written += base + ".bt";
    }
    if (written.empty()) {
      RCLCPP_INFO(get_logger(), "nothing to save");
    } else {
      RCLCPP_INFO(get_logger(), "saved %s: %s", out_dir_.c_str(), written.c_str());
    }
  }

private:
  static std::string nowStamp()
  {
    const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_r(&t, &tm);
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &tm);
    return buffer;
  }

  bool save2d(const std::string & base)
  {
    const auto & info = map2d_->info;
    const size_t width = info.width;
    const size_t height = info.height;
    if (width == 0 || height == 0 || !(info.resolution > 0.0)) {
      return false;
    }
    const size_t cells = width * height;
    if (map2d_->data.size() < cells) {
      RCLCPP_WARN(get_logger(), "occupancy grid data is truncated (%zu < %zu)",
                  map2d_->data.size(), cells);
      return false;
    }
    std::vector<unsigned char> pixels(cells);
    size_t occupied = 0;
    for (size_t row = 0; row < height; ++row) {
      const size_t src = (height - 1 - row) * width;
      for (size_t col = 0; col < width; ++col) {
        const int value = map2d_->data[src + col];
        unsigned char pixel = 205;
        if (value >= 0) {
          double prob = value / 100.0;
          if (negate_) {
            prob = 1.0 - prob;
          }
          if (prob > occupied_thresh_) {
            pixel = 0;
            ++occupied;
          } else if (prob < free_thresh_) {
            pixel = 254;
          }
        }
        pixels[row * width + col] = pixel;
      }
    }
    if (occupied == 0) {
      return false;
    }
    std::ofstream pgm(out_dir_ + "/" + base + ".pgm", std::ios::binary);
    if (!pgm) {
      return false;
    }
    pgm << "P5\n" << width << " " << height << "\n255\n";
    pgm.write(reinterpret_cast<const char *>(pixels.data()), pixels.size());

    char numbers[256];
    std::ofstream yaml(out_dir_ + "/" + base + ".yaml");
    if (!yaml) {
      return false;
    }
    yaml << "image: " << base << ".pgm\n";
    yaml << "mode: trinary\n";
    std::snprintf(numbers, sizeof(numbers), "resolution: %.3f\n", info.resolution);
    yaml << numbers;
    std::snprintf(
      numbers, sizeof(numbers), "origin: [%.3f, %.3f, %.3f]\n",
      info.origin.position.x, info.origin.position.y, info.origin.position.z);
    yaml << numbers;
    yaml << "negate: " << negate_ << "\n";
    std::snprintf(numbers, sizeof(numbers), "occupied_thresh: %.3f\n", occupied_thresh_);
    yaml << numbers;
    std::snprintf(numbers, sizeof(numbers), "free_thresh: %.3f\n", free_thresh_);
    yaml << numbers;
    return true;
  }

  bool save3d(const std::string & base)
  {
    if (!map3d_->binary || map3d_->data.empty()) {
      return false;
    }
    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::binaryMsgToMap(*map3d_));
    if (!tree) {
      return false;
    }
    auto * occ_tree = dynamic_cast<octomap::AbstractOccupancyOcTree *>(tree.get());
    if (!occ_tree) {
      return false;
    }
    if (!occ_tree->writeBinary(out_dir_ + "/" + base + ".bt")) {
      return false;
    }
    return true;
  }

  std::string map2d_topic_;
  std::string map3d_topic_;
  std::string out_dir_;
  std::string prefix_;
  double occupied_thresh_{0.65};
  double free_thresh_{0.196};
  int negate_{0};
  nav_msgs::msg::OccupancyGrid::SharedPtr map2d_;
  octomap_msgs::msg::Octomap::SharedPtr map3d_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub2d_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub3d_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapWatchdog>();
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  while (rclcpp::ok() && !g_stop) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  node->save();
  rclcpp::shutdown();
  return 0;
}
